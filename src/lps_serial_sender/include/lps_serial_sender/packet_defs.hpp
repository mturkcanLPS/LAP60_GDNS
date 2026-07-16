/**
 * packet_defs.hpp — Jetson → otopilot seri paket formati
 *
 * Otopilot firmware ile BIREBIR uyumlu olmali; alan ekleme/cikarma/tasima
 * firmware guncellemesi gerektirir.
 *
 * Paket formatı (packed, little-endian):
 *   header_t           (4 B)
 *   jetsonVariables_t  (158 B): hrvs(16) + apriltag(129) + opticflow(13)
 *   chksum             (1 B)
 *   Toplam: 163 B
 *
 * Zaman tabani — UC ALAN DA AYNI TABANDA, node baslangicinda ~0'dan baslar:
 *   hrvs.timeStamp        : paket gonderim ani (node start'tan beri ms)
 *   apriltag.timeStamp    : goruntunun cekildigi an (ayni taban)
 *   opticflow.timestamp   : goruntunun cekildigi an (ayni taban)
 * Ayni tabanda olduklari icin alanlar arasi fark alinabilir:
 *   sensor gecikmesi = hrvs.timeStamp - apriltag.timeStamp
 * Delta-t HER ZAMAN timestamp farkiyla hesaplanmali (paket varis saatiyle
 * DEGIL — varis araligi OS jitter'i icerir, timestamp icermez).
 * hrvs.frameNumer: monoton paket sayaci, +1'den fazla artis = kayip paket.
 * Acilar: DERECE (quatToRPY radyan dondurur, gonderim oncesi cevrilir).
 */
#pragma once

#include <cstdint>

#define SYNC_BYTE  0xBB
#define MSG_ID     0x42

#pragma pack(1)

struct opticFlow_2D_t {
    float x;
    float y;
};

struct float_3D_t {
    float x;
    float y;
    float z;
};

struct float_angle_t {
    float roll;
    float pitch;
    float yaw;
};

struct AP_position_t {
    bool          valid;
    float_3D_t    pos;
    float_angle_t angle;
};

struct header_t {
    uint8_t sync_byte;
    uint8_t msg_id;
    uint8_t payloadSize;
    uint8_t headerchk;
};

struct hrvsPack_t {
    uint32_t timeStamp;
    uint16_t altitude;
    uint32_t frameNumer;
    int16_t  body_x;
    int16_t  body_y;
    int16_t  body_z;
};

struct apriltagPack_t {
    uint32_t      timeStamp;
    AP_position_t avgPos;
    AP_position_t tag1;   // su an doldurulmuyor (hep 0) — firmware ile uyumlu
    AP_position_t tag2;
    AP_position_t tag3;
    AP_position_t tag4;
};

struct opticFlowPack_t {
    uint32_t       timestamp;
    uint8_t        quality;
    opticFlow_2D_t motionPos;
};

struct jetsonVariables_t {
    hrvsPack_t      hrvs;
    apriltagPack_t  aprtiltag;
    opticFlowPack_t opticflow;
};

struct jetsonOtopilotPack_t {
    header_t          header;
    jetsonVariables_t variables;
    uint8_t           chksum;
};

union jetsonOtopilotPack_ut {
    jetsonOtopilotPack_t pack;
    uint8_t              data[sizeof(jetsonOtopilotPack_t)];
};

#pragma pack()
