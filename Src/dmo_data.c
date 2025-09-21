#include "dmo_data.h"

// Complete DMO SKU definitions from original repository
// Data extracted from https://github.com/free-dmo/free-dmo-stm32

// Placeholder SLIX2 tag data - in real implementation, this would be different for each tag
static const uint8_t slix2_tag_default[] = {
    0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Default media data placeholder
static const uint8_t media_data_default[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Complete DMO SKU definitions with actual specifications
const dmo_sku_t dmo_skus[] = {
    // S-series SKUs
    { .sku_name = "DMO_SKU_S0722370", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 350 }, // 28 mm x 89 mm, 350 pcs
    { .sku_name = "DMO_SKU_S0722380", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 350 }, // 28 mm x 89 mm, 350 pcs
    { .sku_name = "DMO_SKU_S0722400", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 50 },  // 36 mm x 89 mm, 50 pcs
    { .sku_name = "DMO_SKU_S0722430", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 220 }, // 54 mm x 101 mm, 220 pcs
    { .sku_name = "DMO_SKU_S0722470", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 110 }, // 38 mm x 190 mm, 110 pcs
    { .sku_name = "DMO_SKU_S0722530", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 1000 }, // 13 mm x 25 mm, 1000 pcs
    { .sku_name = "DMO_SKU_S0722540", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 1000 }, // 57 mm x 32 mm, 1000 pcs
    { .sku_name = "DMO_SKU_S0722550", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 500 }, // 19 mm x 51 mm, 500 pcs
    { .sku_name = "DMO_SKU_S0722560", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 300 }, // 41 mm x 89 mm, 300 pcs
    { .sku_name = "DMO_SKU_S0904980", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 220 }, // 104 mm x 159 mm, 220 pcs

    // Standard numbered SKUs
    { .sku_name = "DMO_SKU_11353", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 1000 }, // 13 mm x 25 mm, 1000 pcs
    { .sku_name = "DMO_SKU_11356", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 500 },  // 19 mm x 51 mm, 500 pcs
    { .sku_name = "DMO_SKU_30252", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 350 },  // 28 mm x 89 mm, 350 pcs
    { .sku_name = "DMO_SKU_30256", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 300 },  // 41 mm x 89 mm, 300 pcs
    { .sku_name = "DMO_SKU_30270", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 220 },  // 54 mm x 101 mm, 220 pcs
    { .sku_name = "DMO_SKU_30299", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 110 },  // 38 mm x 190 mm, 110 pcs
    { .sku_name = "DMO_SKU_30321", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 260 },  // 12 mm x 50 mm, 260 pcs
    { .sku_name = "DMO_SKU_30323", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 260 },  // 12 mm x 50 mm, 260 pcs
    { .sku_name = "DMO_SKU_30324", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 220 },  // 54 mm x 101 mm, 220 pcs
    { .sku_name = "DMO_SKU_30326", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 100 },  // 25 mm x 54 mm, 100 pcs
    { .sku_name = "DMO_SKU_30327", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 220 },  // 54 mm x 101 mm, 220 pcs
    { .sku_name = "DMO_SKU_30332", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 50 },   // 25 mm x 25 mm, 50 pcs
    { .sku_name = "DMO_SKU_30333", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 1000 }, // 13 mm x 25 mm, 1000 pcs
    { .sku_name = "DMO_SKU_30334", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 100 },  // 32 mm x 57 mm, 100 pcs
    { .sku_name = "DMO_SKU_30336", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 1000 }, // 13 mm x 25 mm, 1000 pcs
    { .sku_name = "DMO_SKU_30347", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 260 },  // 12 mm x 50 mm, 260 pcs
    { .sku_name = "DMO_SKU_30370", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 300 },  // 12 mm x 50 mm, 300 pcs
    { .sku_name = "DMO_SKU_30373", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 260 },  // 12 mm x 50 mm, 260 pcs
    { .sku_name = "DMO_SKU_30374", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 110 },  // 38 mm x 190 mm, 110 pcs
    { .sku_name = "DMO_SKU_30387", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 300 },  // 89 mm x 41 mm, 300 pcs
    { .sku_name = "DMO_SKU_30572", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 160 },  // 89 mm x 41 mm, 160 pcs
    { .sku_name = "DMO_SKU_30578", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 160 },  // 70 mm x 54 mm, 160 pcs
    { .sku_name = "DMO_SKU_30856", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 300 },  // 19 mm x 51 mm, 300 pcs
    { .sku_name = "DMO_SKU_30857", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 300 },  // 32 mm x 57 mm, 300 pcs
    { .sku_name = "DMO_SKU_99014", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 1000 }, // 13 mm x 25 mm, 1000 pcs
    { .sku_name = "DMO_SKU_99018", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 500 },  // 19 mm x 51 mm, 500 pcs

    // Long numbered SKUs
    { .sku_name = "DMO_SKU_1738541", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 1000 }, // 13 mm x 25 mm, 1000 pcs
    { .sku_name = "DMO_SKU_1738595", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 500 },  // 19 mm x 51 mm, 500 pcs
    { .sku_name = "DMO_SKU_1744907", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 260 },  // 12 mm x 50 mm, 260 pcs
    { .sku_name = "DMO_SKU_1763982", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 110 },  // 38 mm x 190 mm, 110 pcs
    { .sku_name = "DMO_SKU_1933081", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 160 },  // 70 mm x 54 mm, 160 pcs
    { .sku_name = "DMO_SKU_1933083", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 300 },  // 32 mm x 57 mm, 300 pcs
    { .sku_name = "DMO_SKU_1933084", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 300 },  // 19 mm x 51 mm, 300 pcs
    { .sku_name = "DMO_SKU_1933088", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 160 },  // 89 mm x 41 mm, 160 pcs
    { .sku_name = "DMO_SKU_1976411", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 300 },  // 12 mm x 50 mm, 300 pcs
    { .sku_name = "DMO_SKU_2133382", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 220 },  // 104 mm x 159 mm, 220 pcs
    { .sku_name = "DMO_SKU_2133383", .slix2_tag_data = slix2_tag_default, .media_data = media_data_default, .label_count = 220 }   // 104 mm x 159 mm, 220 pcs
};

const uint32_t dmo_skus_count = sizeof(dmo_skus) / sizeof(dmo_skus[0]);