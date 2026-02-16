#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <alsa/asoundlib.h>
#include <sndfile.h>
#include <vector>
#include <thread>
#include <atomic>
#include <chrono>
#include <array>

class MultiAmpToneNode : public rclcpp::Node
{
public:
    MultiAmpToneNode() : Node("multi_amp_tone")
    {
        RCLCPP_INFO(this->get_logger(), "MultiAmpToneNode initializing audio subsystem");
        log_alsa_cards();
        subscription_ = create_subscription<std_msgs::msg::UInt8>(
            "/channel_mask", 10,
            std::bind(&MultiAmpToneNode::mask_callback, this, std::placeholders::_1));

        load_tone("/home/harry/glove/src/vibration_manager/audio/vibration_loop.flac");
        open_device(&pcm_a_, "hw:1,0");
        open_device(&pcm_b_, "hw:2,0");

        // Contact topic subscriptions (thumb=0, index=1, middle=2)
        std::array<std::string, 3> finger_names = {"Thumb", "Index", "Middle"};
        std::array<std::string, 3> contact_topics = {
            "/world/shapes/model/glove/link/tl3/sensor/tl3_contact_sensor/contact",
            "/world/shapes/model/glove/link/f1l3/sensor/f1l3_contact_sensor/contact",
            "/world/shapes/model/glove/link/f2l3/sensor/f2l3_contact_sensor/contact"
        };

        for (size_t i = 0; i < 3; ++i) {
            auto sub = create_subscription<ros_gz_interfaces::msg::Contacts>(
                contact_topics[i], 10,
                [this, i](const ros_gz_interfaces::msg::Contacts::SharedPtr msg) {
                    contact_callback(msg, i);
                });
            contact_subs_.push_back(sub);
            RCLCPP_INFO(get_logger(), "Subscribed to %s contacts", finger_names[i].c_str());
        }

        // Watchdog timer (100 Hz check)
        watchdog_timer_ = create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&MultiAmpToneNode::watchdog_callback, this));

        // Initialize contact times to now
        auto now = std::chrono::steady_clock::now();
        for (size_t i = 0; i < 3; ++i) {
            last_contact_time_[i] = now;
        }

        running_ = true;
        RCLCPP_INFO(this->get_logger(), "Starting audio thread");
        audio_thread_ = std::thread(&MultiAmpToneNode::audio_loop, this);
    }

    ~MultiAmpToneNode()
    {
        watchdog_timer_ = nullptr;  // Cancel timer
        RCLCPP_INFO(this->get_logger(), "Stopping audio thread");
        running_ = false;
        audio_thread_.join();

        snd_pcm_close(pcm_a_);
        snd_pcm_close(pcm_b_);
    }

private:

    void mask_callback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        mask_ = msg->data & 0xF;
    }

    void contact_callback(const ros_gz_interfaces::msg::Contacts::SharedPtr msg, size_t index)
    {
        // Record that we received a contact message for this finger
        auto now = std::chrono::steady_clock::now();
        last_contact_time_[index] = now;

        bool was = contact_state_[index];
        bool now_contact = (msg->contacts.size() > 0);

        if (now_contact && !was) {
            RCLCPP_INFO(get_logger(), "COLLISION DETECTED: finger idx=%zu", index);
        }
        if (!now_contact && was) {
            RCLCPP_INFO(get_logger(), "CONTACT RELEASED: finger idx=%zu", index);
        }

        contact_state_[index] = now_contact;
        update_mask_from_contacts();
    }

    void watchdog_callback()
    {
        auto now = std::chrono::steady_clock::now();
        bool changed = false;

        // Check timeout for each finger
        for (size_t i = 0; i < 3; ++i) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_contact_time_[i]).count() / 1000.0;

            if (elapsed > contact_timeout_ && contact_state_[i]) {
                RCLCPP_DEBUG(get_logger(), "CONTACT TIMEOUT: clearing contact for finger idx=%zu", i);
                contact_state_[i] = false;
                changed = true;
            }
        }

        if (changed) {
            update_mask_from_contacts();
        }
    }

    void update_mask_from_contacts()
    {
        // Map contact states to channel mask bits
        // Each finger maps to one bit (thumb=0x1, index=0x2, middle=0x4)
        uint8_t new_mask = 0;
        for (size_t i = 0; i < 3; ++i) {
            if (contact_state_[i]) {
                new_mask |= (1 << i);
            }
        }
        mask_ = new_mask;
    }

    void load_tone(const char* path)
    {
        SF_INFO info;
        SNDFILE* file = sf_open(path, SFM_READ, &info);
        if (!file) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open audio file: %s", path);
            return;
        }

        sample_rate_ = info.samplerate;

        std::vector<float> temp(info.frames * info.channels);
        sf_count_t read = sf_readf_float(file, temp.data(), info.frames);
        sf_close(file);
        if (read != info.frames) {
            RCLCPP_WARN(this->get_logger(), "Expected %ld frames, read %ld", info.frames, read);
        }

        tone_ = temp;
        tone_frames_ = info.frames;
        RCLCPP_INFO(this->get_logger(), "Loaded tone %s (%ld frames, %d Hz)", path, info.frames, info.samplerate);
    }

    void open_device(snd_pcm_t** handle, const char* name)
    {
        int err = snd_pcm_open(handle, name, SND_PCM_STREAM_PLAYBACK, 0);
        if (err < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error opening PCM device %s: %s", name, snd_strerror(err));
            *handle = nullptr;
            return;
        }
        err = snd_pcm_set_params(*handle,
            SND_PCM_FORMAT_FLOAT_LE,
            SND_PCM_ACCESS_RW_INTERLEAVED,
            2,
            sample_rate_,
            1,
            50000); // ~50ms latency
        if (err < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error setting PCM params for %s: %s", name, snd_strerror(err));
            snd_pcm_close(*handle);
            *handle = nullptr;
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Opened PCM device %s (sample_rate=%u)", name, sample_rate_);
    }

    void audio_loop()
    {
        RCLCPP_INFO(this->get_logger(), "audio_loop started");
        const size_t block = 1024;
        std::vector<float> buffer_a(block * 2);
        std::vector<float> buffer_b(block * 2);

        while (running_)
        {
            for (size_t i = 0; i < block; ++i)
            {
                size_t frame = playhead_ % tone_frames_;

                float left  = tone_[frame*2];
                float right = tone_[frame*2+1];

                uint8_t m = mask_.load();

                buffer_a[i*2]   = (m & 0x1) ? left  : 0.0f;
                buffer_a[i*2+1] = (m & 0x2) ? right : 0.0f;

                buffer_b[i*2]   = (m & 0x4) ? left  : 0.0f;
                buffer_b[i*2+1] = (m & 0x8) ? right : 0.0f;

                playhead_++;
            }

            if (pcm_a_) {
                int err = snd_pcm_writei(pcm_a_, buffer_a.data(), block);
                if (err < 0) {
                    RCLCPP_WARN(this->get_logger(), "PCM write error on device A: %s", snd_strerror(err));
                    snd_pcm_prepare(pcm_a_);
                }
            }
            if (pcm_b_) {
                int err = snd_pcm_writei(pcm_b_, buffer_b.data(), block);
                if (err < 0) {
                    RCLCPP_WARN(this->get_logger(), "PCM write error on device B: %s", snd_strerror(err));
                    snd_pcm_prepare(pcm_b_);
                }
            }
        }
        RCLCPP_INFO(this->get_logger(), "audio_loop stopped");
    }

    void log_alsa_cards()
    {
        int card = -1;
        snd_card_next(&card);
        while (card >= 0) {
            char name[32];
            snprintf(name, sizeof(name), "hw:%d", card);
            snd_ctl_t* ctl = nullptr;
            if (snd_ctl_open(&ctl, name, 0) == 0) {
                snd_ctl_card_info_t* info;
                snd_ctl_card_info_alloca(&info);
                if (snd_ctl_card_info(ctl, info) == 0) {
                    const char* id = snd_ctl_card_info_get_id(info);
                    const char* longname = snd_ctl_card_info_get_name(info);
                    RCLCPP_INFO(this->get_logger(), "ALSA card %s: %s", id, longname);
                }
                snd_ctl_close(ctl);
            }
            snd_card_next(&card);
        }
    }

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscription_;

    std::vector<rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr> contact_subs_;
    std::array<bool, 3> contact_state_ = {false, false, false};
    std::array<std::chrono::steady_clock::time_point, 3> last_contact_time_;
    double contact_timeout_ = 0.5;  // seconds
    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    std::vector<float> tone_;
    size_t tone_frames_;
    unsigned int sample_rate_;

    std::atomic<uint8_t> mask_{0xF};
    std::atomic<bool> running_{false};
    uint64_t playhead_ = 0;

    snd_pcm_t* pcm_a_;
    snd_pcm_t* pcm_b_;

    std::thread audio_thread_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MultiAmpToneNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
