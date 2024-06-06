#include "atcacert/atcacert_def.h"

const uint8_t g_csr_template_3_device[] = {
    0x30, 0x81, 0xec, 0x30, 0x81, 0x93, 0x02, 0x01,  0x00, 0x30, 0x31, 0x31, 0x0b, 0x30, 0x09, 0x06,
    0x03, 0x55, 0x04, 0x06, 0x13, 0x02, 0x42, 0x45,  0x31, 0x11, 0x30, 0x0f, 0x06, 0x03, 0x55, 0x04,
    0x0a, 0x0c, 0x08, 0x4b, 0x45, 0x53, 0x20, 0x46,  0x46, 0x46, 0x46, 0x31, 0x0f, 0x30, 0x0d, 0x06,
    0x03, 0x55, 0x04, 0x03, 0x0c, 0x06, 0x64, 0x65,  0x76, 0x31, 0x32, 0x33, 0x30, 0x59, 0x30, 0x13,
    0x06, 0x07, 0x2a, 0x86, 0x48, 0xce, 0x3d, 0x02,  0x01, 0x06, 0x08, 0x2a, 0x86, 0x48, 0xce, 0x3d,
    0x03, 0x01, 0x07, 0x03, 0x42, 0x00, 0x04, 0x90,  0x43, 0x9c, 0xa7, 0xc1, 0x7b, 0x15, 0xdf, 0x6e,
    0xe0, 0x98, 0xbc, 0xe8, 0x1e, 0x62, 0xde, 0xd4,  0x5c, 0x89, 0x37, 0xea, 0x51, 0xc1, 0xa5, 0xff,
    0x43, 0x7e, 0xc7, 0x46, 0x19, 0x27, 0xd2, 0x70,  0x37, 0xaa, 0x43, 0xc0, 0x8b, 0x99, 0x88, 0x02,
    0x29, 0x0b, 0xf9, 0x58, 0x8d, 0xb2, 0xb3, 0x64,  0xfb, 0xc7, 0x08, 0xe2, 0x3a, 0x35, 0xb4, 0xdd,
    0x90, 0x63, 0x8e, 0x55, 0xec, 0xeb, 0xaf, 0xa0,  0x00, 0x30, 0x0a, 0x06, 0x08, 0x2a, 0x86, 0x48,
    0xce, 0x3d, 0x04, 0x03, 0x02, 0x03, 0x48, 0x00,  0x30, 0x45, 0x02, 0x20, 0x5a, 0x07, 0x84, 0xa1,
    0x73, 0xdf, 0x8b, 0xb3, 0x15, 0x86, 0x59, 0xe7,  0x66, 0xdf, 0xd2, 0x7c, 0xd7, 0x0f, 0xae, 0xf8,
    0xbe, 0xb0, 0x7d, 0x3e, 0xc0, 0x19, 0x8e, 0x7f,  0x55, 0x82, 0xf0, 0xeb, 0x02, 0x21, 0x00, 0x9b,
    0x7c, 0x68, 0x98, 0x34, 0x5d, 0xf6, 0x3b, 0xa1,  0x61, 0xfd, 0xee, 0xcf, 0x94, 0x64, 0x70, 0x58,
    0xe3, 0xd5, 0xc5, 0x8f, 0xe5, 0x5a, 0x57, 0x7b,  0x45, 0x23, 0x29, 0x88, 0xad, 0x67, 0xb8
};

const atcacert_def_t g_csr_def_3_device = {
    .type                   = CERTTYPE_X509,
    .template_id            = 3,
    .chain_id               = 0,
    .private_key_slot       = 0,
    .sn_source              = SNSRC_PUB_KEY_HASH,
    .cert_sn_dev_loc        = {
        .zone      = DEVZONE_NONE,
        .slot      = 0,
        .is_genkey = 0,
        .offset    = 0,
        .count     = 0
    },
    .issue_date_format      = DATEFMT_RFC5280_UTC,
    .expire_date_format     = DATEFMT_RFC5280_UTC,
    .tbs_cert_loc           = {
        .offset = 3,
        .count  = 150
    },
    .expire_years           = 0,
    .public_key_dev_loc     = {
        .zone      = DEVZONE_NONE,
        .slot      = 0,
        .is_genkey = 1,
        .offset    = 0,
        .count     = 64
    },
    .comp_cert_dev_loc      = {
        .zone      = DEVZONE_NONE,
        .slot      = 0,
        .is_genkey = 0,
        .offset    = 0,
        .count     = 0
    },
    .std_cert_elements      = {
        { // STDCERT_PUBLIC_KEY
            .offset = 87,
            .count  = 64
        },
        { // STDCERT_SIGNATURE
            .offset = 165,
            .count  = 74
        },
        { // STDCERT_ISSUE_DATE
            .offset = 0,
            .count  = 0
        },
        { // STDCERT_EXPIRE_DATE
            .offset = 0,
            .count  = 0
        },
        { // STDCERT_SIGNER_ID
            .offset = 0,
            .count  = 0
        },
        { // STDCERT_CERT_SN
            .offset = 0,
            .count  = 0
        },
        { // STDCERT_AUTH_KEY_ID
            .offset = 0,
            .count  = 0
        },
        { // STDCERT_SUBJ_KEY_ID
            .offset = 0,
            .count  = 0
        }
    },
    .cert_elements          = NULL,
    .cert_elements_count    = 0,
    .cert_template          = g_csr_template_3_device,
    .cert_template_size     = sizeof(g_csr_template_3_device)
};