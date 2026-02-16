#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <alsa/asoundlib.h>
#include <sndfile.h>
#include <vector>
#include <thread>
#include <atomic>

class MultiAmpToneNode : public rclcpp::Node
{
public:
    MultiAmpToneNode() : Node("multi_amp_tone")
    {
        subscription_ = create_subscription<std_msgs::msg::UInt8>(
            "/channel_mask", 10,
            std::bind(&MultiAmpToneNode::mask_callback, this, std::placeholders::_1));

        load_tone("~/glove/src/vibration_manager/audio/vibration_loop.flac");
        open_device(&pcm_a_, "hw:1,0");
        open_device(&pcm_b_, "hw:2,0");

        running_ = true;
        audio_thread_ = std::thread(&MultiAmpToneNode::audio_loop, this);
    }

    ~MultiAmpToneNode()
    {
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

    void load_tone(const char* path)
    {
        SF_INFO info;
        SNDFILE* file = sf_open(path, SFM_READ, &info);

        sample_rate_ = info.samplerate;

        std::vector<float> temp(info.frames * info.channels);
        sf_readf_float(file, temp.data(), info.frames);
        sf_close(file);

        tone_ = temp;
        tone_frames_ = info.frames;
    }

    void open_device(snd_pcm_t** handle, const char* name)
    {
        snd_pcm_open(handle, name, SND_PCM_STREAM_PLAYBACK, 0);
        snd_pcm_set_params(*handle,
            SND_PCM_FORMAT_FLOAT_LE,
            SND_PCM_ACCESS_RW_INTERLEAVED,
            2,
            sample_rate_,
            1,
            50000); // ~50ms latency
    }

    void audio_loop()
    {
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

            snd_pcm_writei(pcm_a_, buffer_a.data(), block);
            snd_pcm_writei(pcm_b_, buffer_b.data(), block);
        }
    }

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscription_;

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
