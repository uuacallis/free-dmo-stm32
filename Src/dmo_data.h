#ifndef DMO_DATA_H_
#define DMO_DATA_H_

#include <stdint.h>

typedef struct {
    const char* sku_name;
    const uint8_t* slix2_tag_data;
    const uint8_t* media_data;
    const uint16_t label_count;
} dmo_sku_t;

// EXTERNAL DECLARATIONS: Tell the compiler these exist and are defined elsewhere.
extern const dmo_sku_t dmo_skus[];
extern const uint32_t dmo_skus_count;

#endif /* DMO_DATA_H_ */


