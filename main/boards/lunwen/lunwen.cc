#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/lcd_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "lamp_controller.h"
#include "led/single_led.h"

#include "emoji_controller.h"
#include "emotion_response_controller.h"

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <driver/spi_common.h>

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <atomic>
#include <mutex>
#include <string>

#if defined(LCD_TYPE_ILI9341_SERIAL)
#include "esp_lcd_ili9341.h"
#endif

#if defined(LCD_TYPE_GC9A01_SERIAL)
#include "esp_lcd_gc9a01.h"
static const gc9a01_lcd_init_cmd_t gc9107_lcd_init_cmds[] = {
    //  {cmd, { data }, data_size, delay_ms}
    {0xfe, (uint8_t[]){0x00}, 0, 0},
    {0xef, (uint8_t[]){0x00}, 0, 0},
    {0xb0, (uint8_t[]){0xc0}, 1, 0},
    {0xb1, (uint8_t[]){0x80}, 1, 0},
    {0xb2, (uint8_t[]){0x27}, 1, 0},
    {0xb3, (uint8_t[]){0x13}, 1, 0},
    {0xb6, (uint8_t[]){0x19}, 1, 0},
    {0xb7, (uint8_t[]){0x05}, 1, 0},
    {0xac, (uint8_t[]){0xc8}, 1, 0},
    {0xab, (uint8_t[]){0x0f}, 1, 0},
    {0x3a, (uint8_t[]){0x05}, 1, 0},
    {0xb4, (uint8_t[]){0x04}, 1, 0},
    {0xa8, (uint8_t[]){0x08}, 1, 0},
    {0xb8, (uint8_t[]){0x08}, 1, 0},
    {0xea, (uint8_t[]){0x02}, 1, 0},
    {0xe8, (uint8_t[]){0x2A}, 1, 0},
    {0xe9, (uint8_t[]){0x47}, 1, 0},
    {0xe7, (uint8_t[]){0x5f}, 1, 0},
    {0xc6, (uint8_t[]){0x21}, 1, 0},
    {0xc7, (uint8_t[]){0x15}, 1, 0},
    {0xf0,
    (uint8_t[]){0x1D, 0x38, 0x09, 0x4D, 0x92, 0x2F, 0x35, 0x52, 0x1E, 0x0C,
                0x04, 0x12, 0x14, 0x1f},
    14, 0},
    {0xf1,
    (uint8_t[]){0x16, 0x40, 0x1C, 0x54, 0xA9, 0x2D, 0x2E, 0x56, 0x10, 0x0D,
                0x0C, 0x1A, 0x14, 0x1E},
    14, 0},
    {0xf4, (uint8_t[]){0x00, 0x00, 0xFF}, 3, 0},
    {0xba, (uint8_t[]){0xFF, 0xFF}, 2, 0},
};
#endif

#define TAG "Lunwen"

#if defined(LVGL_VERSION_MAJOR) && (LVGL_VERSION_MAJOR >= 9)
#define LUNWEN_SCREEN_ACTIVE()      lv_screen_active()
#define LUNWEN_SCREEN_LOAD(screen)  lv_screen_load(screen)
#else
#define LUNWEN_SCREEN_ACTIVE()      lv_scr_act()
#define LUNWEN_SCREEN_LOAD(screen)  lv_scr_load(screen)
#endif

#ifndef EMOJI_FACE_WIDTH
#define EMOJI_FACE_WIDTH  128
#endif
#ifndef EMOJI_FACE_HEIGHT
#define EMOJI_FACE_HEIGHT 64
#endif


#ifndef EMOJI_FACE_AUTO_CENTER
#define EMOJI_FACE_AUTO_CENTER 1
#endif

class Lunwen;

class LunwenDisplay : public SpiLcdDisplay {
public:
    LunwenDisplay(Lunwen* board, esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                  int width, int height, int offset_x, int offset_y,
                  bool mirror_x, bool mirror_y, bool swap_xy)
        : SpiLcdDisplay(panel_io, panel, width, height, offset_x, offset_y,
                        mirror_x, mirror_y, swap_xy),
          board_(board) {}

    void SetEmotion(const char* emotion) override;

    void SetChatMessage(const char* role, const char* content) override;

private:
    Lunwen* board_;
    bool processing_ai_response_ = false;  
};

class Lunwen : public WifiBoard {
private:
    Button boot_button_;
    LcdDisplay* display_;

   
    EmojiController* emoji_controller_ = nullptr;              
    EmotionResponseController* emotion_controller_ = nullptr;  

    bool controllers_ready_ = false;           
    std::atomic<bool> is_emoji_mode_{false};   
    lv_obj_t* chat_screen_ = nullptr;         
    std::string last_ai_response_;             
    std::string last_expression_;              

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
#if defined(LCD_TYPE_ILI9341_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel));
#elif defined(LCD_TYPE_GC9A01_SERIAL)
        
        gc9a01_vendor_config_t gc9107_vendor_config = {
            .init_cmds = gc9107_lcd_init_cmds,
            .init_cmds_size = sizeof(gc9107_lcd_init_cmds) / sizeof(gc9a01_lcd_init_cmd_t),
        };
        panel_config.vendor_config = &gc9107_vendor_config;
        ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(panel_io, &panel_config, &panel));
#else
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));
#endif

        esp_lcd_panel_reset(panel);

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
       
        display_ = new LunwenDisplay(this, panel_io, panel,
                                     DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }


    void InitializeEmoji() {
        emoji_controller_ = new EmojiController(display_);

        emoji_controller_->Initialize();   

       
        emotion_controller_ = new EmotionResponseController(emoji_controller_, GetAudioCodec());
        emotion_controller_->Initialize();

        {
            DisplayLockGuard lock(display_);
            emoji_controller_->CreateEmojiScreen();
            CenterEmojiFace();
        }

        emoji_controller_->SetRandomAnimationEnabled(false);
        emoji_controller_->EyeCenter();

        controllers_ready_ = true;
        ESP_LOGI(TAG, "表情控制器初始化完成");
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });

      
        boot_button_.OnLongPress([this]() {
            ToggleEmojiMode();
        });
    }

    void InitializeTools() {
        
#ifdef LAMP_GPIO
        if (LAMP_GPIO != GPIO_NUM_NC) {
            static LampController lamp(LAMP_GPIO);
        }
#endif
    }

    static std::mutex& ActionMutex() {
        static std::mutex mutex;
        return mutex;
    }

    static std::mutex& ResponseMutex() {
        static std::mutex mutex;
        return mutex;
    }

    bool HasLastResponse() {
        std::lock_guard<std::mutex> lock(ResponseMutex());
        return !last_ai_response_.empty();
    }

    void CenterEmojiFace() {
#if EMOJI_FACE_AUTO_CENTER
        if (emoji_controller_ == nullptr) {
            return;
        }
        lv_obj_t* screen = emoji_controller_->GetEmojiScreen();
        if (screen == nullptr) {
            return;
        }
        const int dx = (DISPLAY_WIDTH - EMOJI_FACE_WIDTH) / 2;
        const int dy = (DISPLAY_HEIGHT - EMOJI_FACE_HEIGHT) / 2;
        if (dx == 0 && dy == 0) {
            return;
        }

        for (uint32_t i = 0;; ++i) {
            lv_obj_t* child = lv_obj_get_child(screen, i);
            if (child == nullptr) {
                break;
            }
            lv_obj_set_style_translate_x(child, dx, 0);
            lv_obj_set_style_translate_y(child, dy, 0);
        }
#endif
    }


    void EnterEmojiMode() {
        if (is_emoji_mode_ || emoji_controller_ == nullptr) {
            return;
        }

        {
            DisplayLockGuard lock(display_);
            if (chat_screen_ == nullptr) {
                chat_screen_ = LUNWEN_SCREEN_ACTIVE();
            }
            lv_obj_t* emoji_screen = emoji_controller_->CreateEmojiScreen();
            if (emoji_screen != nullptr) {
                CenterEmojiFace();
                LUNWEN_SCREEN_LOAD(emoji_screen);
            }
        }

        is_emoji_mode_ = true;
        emoji_controller_->EyeCenter();

        emoji_controller_->StartBlinkTimer();
        emoji_controller_->SetRandomAnimationEnabled(true); 

        GetDisplay()->ShowNotification("表情模式");
        ESP_LOGI(TAG, "已进入表情模式");
    }


    void ExitEmojiMode() {
        if (!is_emoji_mode_ || emoji_controller_ == nullptr) {
            return;
        }

        emoji_controller_->StopBlinkTimer();           
        emoji_controller_->SetRandomAnimationEnabled(false);
        emoji_controller_->ClearAnimationQueue();    
        emoji_controller_->EyeCenter(false);

        {
            DisplayLockGuard lock(display_);
            if (chat_screen_ != nullptr) {
                LUNWEN_SCREEN_LOAD(chat_screen_);
            }
        
        }

        is_emoji_mode_ = false;
        GetDisplay()->ShowNotification("对话模式");
        ESP_LOGI(TAG, "已退出表情模式");
    }

    void ToggleEmojiMode() {
    
        if (Application::GetInstance().GetDeviceState() == kDeviceStateStarting) {
            return;
        }
        if (is_emoji_mode_) {
            ExitEmojiMode();
        } else {
            EnterEmojiMode();
        }
    }


    void ProcessAIResponseInternal(const char* message) {
        if (message == nullptr || message[0] == '\0' || emotion_controller_ == nullptr) {
            return;
        }

        ESP_LOGI(TAG, "分析 AI 回复: %s", message);

      
        std::lock_guard<std::mutex> lock(ActionMutex());

  
        const bool explicit_emotion = (strstr(message, "[emotion:") != nullptr);
        const std::string before = emotion_controller_->GetCurrentEmotion();

    
        emotion_controller_->ProcessAIResponse(message);

        const std::string after = emotion_controller_->GetCurrentEmotion();
        if (!explicit_emotion && after == "neutral" && before == last_expression_ &&
            !last_expression_.empty() && last_expression_ != "neutral") {
        
            ESP_LOGI(TAG, "文本无明确情绪，恢复上一次表情: %s", last_expression_.c_str());
            emotion_controller_->TriggerEmotion(last_expression_);
        }
    }

    struct AIResponseJob {
        Lunwen* board;
        std::string message;
    };

    static void ProcessAIResponseTask(void* arg) {
        auto* job = static_cast<AIResponseJob*>(arg);
        if (job != nullptr) {
            if (job->board != nullptr) {
                job->board->ProcessAIResponseInternal(job->message.c_str());
            }
            delete job;
        }
        vTaskDelete(nullptr);
    }

 
    struct EmotionJob {
        Lunwen* board;
        std::string emotion;  
    };

    static void EmotionTask(void* arg) {
        auto* job = static_cast<EmotionJob*>(arg);
        if (job != nullptr) {
            if (job->board != nullptr && job->board->emotion_controller_ != nullptr) {
                std::lock_guard<std::mutex> lock(ActionMutex());
              
                job->board->last_expression_ = job->emotion;
                job->board->emotion_controller_->TriggerEmotion(job->emotion);
            }
            delete job;
        }
        vTaskDelete(nullptr);
    }


    void PostEmotionAction(const std::string& emotion) {
        if (emotion_controller_ == nullptr) {
            return;
        }
        auto* job = new EmotionJob{this, emotion};
        if (xTaskCreate(EmotionTask, "lunwen_emotion", 4096, job, 1, nullptr) != pdPASS) {
            ESP_LOGW(TAG, "创建情绪动作任务失败: %s", emotion.c_str());
            delete job;
        }
    }

    static void StateMonitorTask(void* arg) {
        auto* board = static_cast<Lunwen*>(arg);
        if (board != nullptr) {
            board->MonitorDeviceState();
        }
        vTaskDelete(nullptr);
    }

    void MonitorDeviceState() {
        srand((unsigned)time(nullptr));

        DeviceState last_state = kDeviceStateIdle;
        TickType_t last_active_end = xTaskGetTickCount(); 
        bool in_conversation = false;

        while (true) {
            DeviceState state = Application::GetInstance().GetDeviceState();

            if (state != last_state) {
                ESP_LOGI(TAG, "设备状态: %d -> %d", (int)last_state, (int)state);
            }

            const bool talking = (state == kDeviceStateSpeaking || state == kDeviceStateListening);
            const bool was_talking = (last_state == kDeviceStateSpeaking || last_state == kDeviceStateListening);

   
            if (talking && !in_conversation) {
                in_conversation = true;
                {
                    std::lock_guard<std::mutex> lock(ResponseMutex());
                    last_ai_response_.clear();
                }
                if (emoji_controller_ != nullptr) {
                    emoji_controller_->SetRandomAnimationEnabled(false);
                    emoji_controller_->ClearAnimationQueue();
                }
            }

    
            if (was_talking && !talking) {
                last_active_end = xTaskGetTickCount();
                if (last_state == kDeviceStateSpeaking && !HasLastResponse() && controllers_ready_) {
                    static const char* kRandom[] = {"happy", "surprised", "confused", "neutral"};
                    TriggerEmotion(kRandom[rand() % 4]);
                }
            }

          
            if (in_conversation && state == kDeviceStateIdle &&
                (xTaskGetTickCount() - last_active_end) > pdMS_TO_TICKS(3000)) {
                in_conversation = false;
                if (emoji_controller_ != nullptr && is_emoji_mode_) {
                    emoji_controller_->SetRandomAnimationEnabled(true);
                }
                TriggerEmotion("neutral");
            }

            last_state = state;
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

public:
    Lunwen() :
        boot_button_(BOOT_BUTTON_GPIO) {
        InitializeSpi();
        InitializeLcdDisplay();
        InitializeEmoji();    
        InitializeButtons();
        InitializeTools();
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            GetBacklight()->RestoreBrightness();
        }

        
        xTaskCreate(StateMonitorTask, "lunwen_state", 8192, this, 1, nullptr);
    }

    ~Lunwen() override = default;

 
    void ProcessAIResponse(const char* message) {
        if (message == nullptr || message[0] == '\0' || !controllers_ready_) {
            return;
        }

       
        {
            std::lock_guard<std::mutex> lock(ResponseMutex());
            if (last_ai_response_ == message) {
                return;
            }
            last_ai_response_ = message;
        }

        auto* job = new AIResponseJob{this, std::string(message)};
        if (xTaskCreate(ProcessAIResponseTask, "lunwen_ai", 8192, job, 1, nullptr) != pdPASS) {
            ESP_LOGW(TAG, "创建 AI 回复处理任务失败");
            delete job;
        }
    }


    void TriggerEmotion(const char* framework_emotion) {
        if (framework_emotion == nullptr || framework_emotion[0] == '\0' || !controllers_ready_) {
            return;
        }

        
        static const struct {
            const char* framework;
            const char* local;
        } kEmotionMap[] = {
            {"happy", "happy"},       {"laughing", "laughing"}, {"funny", "funny"},
            {"sad", "sad"},           {"crying", "cry"},        {"angry", "anger"},
            {"surprised", "surprise"},{"shocked", "shocked"},   {"confused", "confused"},
            {"thinking", "thinking"}, {"embarrassed", "awkward"},{"sleepy", "sleep"},
            {"winking", "blink"},     {"cool", "cool"},         {"confident", "confident"},
            {"relaxed", "relaxed"},   {"loving", "loving"},     {"kissy", "kissy"},
            {"delicious", "delicious"},{"silly", "silly"},      {"neutral", "neutral"},
        };

        std::string local_emotion = "neutral";
        for (const auto& item : kEmotionMap) {
            if (strcmp(framework_emotion, item.framework) == 0) {
                local_emotion = item.local;
                break;
            }
        }

        ESP_LOGI(TAG, "框架情绪 %s -> 表情 %s", framework_emotion, local_emotion.c_str());
        PostEmotionAction(local_emotion);
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Backlight* GetBacklight() override {
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
            return &backlight;
        }
        return nullptr;
    }
};


void LunwenDisplay::SetChatMessage(const char* role, const char* content) {
    
    if (processing_ai_response_) {
        SpiLcdDisplay::SetChatMessage(role, content);
        return;
    }

    processing_ai_response_ = true;
    SpiLcdDisplay::SetChatMessage(role, content);

    if (role != nullptr && content != nullptr && content[0] != '\0' &&
        strcmp(role, "assistant") == 0 && board_ != nullptr) {
        board_->ProcessAIResponse(content);
    }

    processing_ai_response_ = false;
}


void LunwenDisplay::SetEmotion(const char* emotion) {
    if (emotion == nullptr) {
        return;
    }
    SpiLcdDisplay::SetEmotion(emotion);

    if (processing_ai_response_ || board_ == nullptr) {
        return;
    }
    board_->TriggerEmotion(emotion);
}

DECLARE_BOARD(Lunwen);
