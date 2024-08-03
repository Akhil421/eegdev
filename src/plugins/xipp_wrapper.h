#ifndef XIPP_H
#define XIPP_H

#ifdef __cplusplus
extern "C" {
#endif

    #include <stdint.h>
    #include <stdbool.h>

    #define STRLEN_LABEL 128

    typedef struct {
        int totalSizeQuads;
        int processorID;
        int moduleID;
        int outputStreamID;
        unsigned int timeDooDas;
        int streamType;
        int flags;
        int class_id;
        int sample_count;
        int16_t* i16;
    } XippSegmentDataPacketFixed_t;

    typedef struct {
        int totalSizeQuads;
        int processorID;
        int moduleID;
        int outputStreamID;
        unsigned int timeDooDas;
        int streamType;
        int flags;
        int reason;
        int reserved;
        int parallel;
        int sma1;
        int sma2;
        int sma3;
        int sma4;
        int PADDING;
    } XippDigitalEventPacket_t;

    typedef struct {
        int elec;
        size_t length;
        uint_fast16_t* array;
    } StimSeq_t;

    typedef struct {
        double b0;
        double b1;
        double a1;
        double a2;
    } SosStage_t;

    typedef struct {
        char* label;
        double center;
        double lowCutoff;
        double highCutoff;
        int centerOrder;
        int centerFlags;
        int lowOrder;
        int lowFlags;
        int highOrder;
        int highFlags;
        int maxStages;
        int numStages;
        SosStage_t* stages;
    } SosFilterDesc_t;

    typedef struct {
        int command;
        int length;
        uint16_t* data;
    } XippTransceiverCommand_t;

    typedef struct {
        double current;
        double voltage;
        double power;
    } XippSensor_t;

    typedef struct {
        int timestamp;
        int* data;
    } XippCalib_t;

    typedef struct {
        int status;
        int status_size;
        int file_name_base;
        int file_name_base_size;
        int error_msg;
        int error_msg_size;
        int auto_stop_time;
        int auto_incr;
        int incr_num;
    } XippTrialDescriptor_t;

    int xl_open_tcp();
    int xl_open_udp();
    int xl_close();
    int xl_error_string(char* BUF, int arg2, int arg3);
    int xl_cont_raw(unsigned int* arg1, float* arg2, unsigned int arg3, const unsigned int* arg4, unsigned int arg5, unsigned int arg6);
    int xl_cont_hires(unsigned int* arg1, float* arg2, unsigned int arg3, const unsigned int* arg4, unsigned int arg5, unsigned int arg6);
    int xl_cont_hifreq(unsigned int* arg1, float* arg2, unsigned int arg3, const unsigned int* arg4, unsigned int arg5, unsigned int arg6);
    int xl_cont_lfp(unsigned int* arg1, float* arg2, unsigned int arg3, const unsigned int* arg4, unsigned int arg5, unsigned int arg6);
    int xl_cont_emg(unsigned int* arg1, float* arg2, unsigned int arg3, const unsigned int* arg4, unsigned int arg5, unsigned int arg6);
    int xl_cont_status(unsigned int* arg1, float* arg2, unsigned int arg3, const unsigned int* arg4, unsigned int arg5, unsigned int arg6);
    int xl_stim_data(XippSegmentDataPacketFixed_t arg1, int* arg2, unsigned int arg3, unsigned int arg4);
    int xl_spk_data(XippSegmentDataPacketFixed_t arg1, int* arg2, unsigned int arg3, unsigned int arg4);
    int xl_spk_thresh(float* arg1, float* arg2, unsigned int arg3);
    int xl_spk_thresh_set(unsigned int arg1, float arg2, float arg3);
    int xl_digin(XippDigitalEventPacket_t* arg1, int* arg2, unsigned int arg3);
    int xl_digout(const unsigned int* arg1, const unsigned int* arg2, int len);
    int xl_list_elec(unsigned int* arg1, unsigned int arg2, const char* arg3);
    unsigned int xl_time();
    int xl_get_fe(int arg1);
    int xl_get_fe_streams(char (*STR_ARRAY)[STRLEN_LABEL], int arg2, int arg3);
    int xl_nip_serial(char* BUF, int arg2);
    int xl_lib_version(char* BUF, int arg2);
    int xl_xipp_version(char* BUF, int arg2);
    int xl_nipexec_version(char* BUF, int arg2);
    int xl_fe_version(char* BUF, int arg2, int arg3);
    int xl_signal(int arg1, const char* STR);
    int xl_signal_raw(int arg1);
    int xl_signal_lfp(int arg1);
    int xl_signal_spk(int arg1);
    int xl_signal_stim(int arg1);
    int xl_signal_set(int arg1, const char* STR, int arg3);
    int xl_signal_set_raw(int arg1, int arg2);
    int xl_signal_set_lfp(int arg1, int arg2);
    int xl_signal_set_spk(int arg1, int arg2);
    int xl_signal_set_stim(int arg1, int arg2);
    int xl_signal_save(int arg1, const char* STR);
    int xl_signal_save_set(int arg1, const char* STR, int arg3);
    int xl_stim_enable_set(int arg1);
    int xl_stim_enable();
    int xl_stim_get_res(int arg1, int* arg2);
    int xl_stim_set_res(int arg1, int arg2);
    int xl_stim_header_word(int arg1, int arg2, int arg3);
    int xl_stim_word(int arg1, int arg2, int arg3, int arg4, int arg5, int arg6, int arg7);
    int xl_stim_seq(StimSeq_t arg1, int arg2);
    int xl_filter_list_names(char (*STR_ARRAY)[STRLEN_LABEL], int arg2, int arg3);
    int xl_filter_list_selection(int* arg1, int elec, const char* STR);
    int xl_filter_set(int arg1, const char* STR, int arg3);
    SosFilterDesc_t* xl_filter_get_desc(int arg1, const char* STR, int arg3);
    int xl_filter_set_custom(int arg1, const char* STR, SosFilterDesc_t arg3);
    int xl_imp(float* imp, const unsigned int* chan, int arg3);
    int xl_fast_settle_get_choices(int arg1, const char* STR, char (*STR_ARRAY)[STRLEN_LABEL], int arg4, int* arg5);
    int xl_fast_settle_get_duration(int arg1, const char* STR, float* arg3);
    int xl_fast_settle(int arg1, const char* STR, int* arg3, float* arg4);
    int xl_transceiver_connected(int arg1, int* arg2);
    int xl_transceiver_enable(int arg1, int arg2);
    int xl_transceiver_status(uint16_t* arg1, int arg2);
    int xl_transceiver_command(int arg1, XippTransceiverCommand_t arg2);
    int xl_transceiver_cmd_response(uint16_t* arg1, int arg2);
    int xl_hw_ground(unsigned int arg1, bool arg2);
    int xl_hw_reference(unsigned int arg1, bool arg2);
    int xl_internal_battery(int *arg1);
    int xl_external_battery(int *arg1);
    int xl_external_battery_detected(int *arg1);
    int xl_wall_sensor(XippSensor_t arg1);
    int xl_vdd_sensor(XippSensor_t arg1);
    int xl_audio_tone_set(unsigned int *arg1, unsigned int *arg2);
    int xl_stim_calib_set(int arg1, XippCalib_t arg2);
    XippCalib_t* xl_stim_calib_get(int arg1);
    int xl_stim_calib_start(int arg1);

    typedef enum {
        XL_BUTTON_STOP_STIM = 1,
        XL_BUTTON_EVENT,
        XL_BUTTON_F1,
        XL_BUTTON_F2,
    } XL_BUTTON_ID;

    typedef enum {
        XL_LED_EVENT = 2,
        XL_LED_F1,
        XL_LED_F2,
        XL_LED_PORTA,
        XL_LED_PORTB,
        XL_LED_PORTC,
        XL_LED_PORTD,
    } XL_LED_ID;

    int xl_button_get_count(int *arg1, XL_BUTTON_ID id);
    int xl_button_set_monitor(int arg1, XL_BUTTON_ID id);
    int xl_led_set(int arg1, XL_LED_ID id);
    int xl_led_set_monitor(int arg1, XL_LED_ID id);
    int xl_led(int *arg1, XL_LED_ID id);
    int xl_processor_restart_software();

    int xl_error_check();

    typedef enum {
        DEFAULT         = 0,
        PREFER_WIRED    = 0,
        PREFER_WIRELESS = 1,
        WIRED_ONLY      = 2,
        WIRELESS_ONLY   = 3,
    } XippConnectionPolicy;

    int xl_operator_add(uint32_t* arg1, const char* arg2, const XippConnectionPolicy* arg3, const int* arg4);
    int xl_operator_lookup(uint32_t* arg1, const char* arg2);
    int xl_trial2(uint32_t arg1, const XippTrialDescriptor_t arg2, const XippTrialDescriptor_t arg3);

#ifdef __cplusplus
}
#endif

#endif /* XIPP_H */
