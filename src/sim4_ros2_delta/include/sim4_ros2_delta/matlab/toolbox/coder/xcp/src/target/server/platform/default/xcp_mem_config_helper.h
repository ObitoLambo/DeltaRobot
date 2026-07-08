/*
* Copyright 2016-2022 The MathWorks, Inc.
*
* File: xcp_mem_config_helper.h
*
* Abstract:
*  The file contains checks to ensure the XCP_MEM_BLOCK_x_SIZE and 
*  XCP_MEM_BLOCK_x_NUMBER macros are valid and that their default value is 0
*/

#ifndef XCP_MEM_CONFIG_HELPER_H
#define XCP_MEM_CONFIG_HELPER_H

/* These macros trigger a compilation error when expr is false and
   print assert_failed__msg */
#define CA_EXP_CONCAT(assert_failed__, msg) assert_failed__##msg
#define CA_STATIC_ASSERT(expr, msg) \
    typedef char CA_EXP_CONCAT(assert_failed_, msg) [(expr) ? (+1) : (-1)]

/**
 * Size of a pointer, all block sizes must be larger than this value
 **/
#define XCP_MEM_SIZEOF_POINTER (sizeof(void*))


/* Block set 1 */
#if (defined(XCP_MEM_BLOCK_1_SIZE) ^ defined(XCP_MEM_BLOCK_1_NUMBER))
/* only one of the macros is defined */
#error Error setting XCP_MEM_BLOCK_1_SIZE and XCP_MEM_BLOCK_1_NUMBER
#endif

#if (!defined(XCP_MEM_BLOCK_1_SIZE) & !defined(XCP_MEM_BLOCK_1_NUMBER)) || (XCP_MEM_BLOCK_1_SIZE == 0)
/* both macros are undefined, default to 0 */
#define XCP_MEM_BLOCK_1_SIZE    0
#define XCP_MEM_BLOCK_1_NUMBER  0
#else
/* both macros are defined */
/* Check the condition that XCP_ALIGNED(XCP_MEM_BLOCK_1_SIZE) is large enough to hold a pointer */
#define XCP_MEM_ERROR_MESSAGE_1 XCP_MEM_BLOCK_1_SIZE_is_not_large_enough_to_hold_a_pointer
CA_STATIC_ASSERT((XCP_ALIGNED(XCP_MEM_BLOCK_1_SIZE) >= XCP_MEM_SIZEOF_POINTER), XCP_MEM_ERROR_MESSAGE_1);

#endif


/* Block set 2 */
#if (defined(XCP_MEM_BLOCK_2_SIZE) ^ defined(XCP_MEM_BLOCK_2_NUMBER))
#error Error setting XCP_MEM_BLOCK_2_SIZE and XCP_MEM_BLOCK_2_NUMBER
#endif

#if (!defined(XCP_MEM_BLOCK_2_SIZE) & !defined(XCP_MEM_BLOCK_2_NUMBER)) || (XCP_MEM_BLOCK_2_SIZE == 0)
#define XCP_MEM_BLOCK_2_SIZE    0
#define XCP_MEM_BLOCK_2_NUMBER  0
#else

/* If we get here, both macros for set 2 are defined*/
#if ( XCP_MEM_BLOCK_1_NUMBER == 0 )
#error Error: XCP_MEM_BLOCK_2_SIZE and XCP_MEM_BLOCK_2_NUMBER are set but not XCP_MEM_BLOCK_1_SIZE and XCP_MEM_BLOCK_1_NUMBER
#endif

/* Check the condition that XCP_ALIGNED(XCP_MEM_BLOCK_2_SIZE) is large enough to hold a pointer */
#define XCP_MEM_ERROR_MESSAGE_2 XCP_MEM_BLOCK_2_SIZE_is_not_large_enough_to_hold_a_pointer
CA_STATIC_ASSERT((XCP_ALIGNED(XCP_MEM_BLOCK_2_SIZE) >= XCP_MEM_SIZEOF_POINTER), XCP_MEM_ERROR_MESSAGE_2);

#endif


/* Block set 3 */
#if (defined(XCP_MEM_BLOCK_3_SIZE) ^ defined(XCP_MEM_BLOCK_3_NUMBER))
#error Error setting XCP_MEM_BLOCK_3_SIZE and XCP_MEM_BLOCK_3_NUMBER
#endif

#if (!defined(XCP_MEM_BLOCK_3_SIZE) & !defined(XCP_MEM_BLOCK_3_NUMBER)) || (XCP_MEM_BLOCK_3_SIZE == 0)
#define XCP_MEM_BLOCK_3_SIZE    0
#define XCP_MEM_BLOCK_3_NUMBER  0
#else

/* If we get here, both macros for set 3 are defined*/
#if ( XCP_MEM_BLOCK_2_NUMBER == 0 )
#error Error: XCP_MEM_BLOCK_3_SIZE and XCP_MEM_BLOCK_3_NUMBER are set but not XCP_MEM_BLOCK_2_SIZE and XCP_MEM_BLOCK_2_NUMBER
#endif

/* Check the condition that XCP_ALIGNED(XCP_MEM_BLOCK_3_SIZE) is large enough to hold a pointer */
#define XCP_MEM_ERROR_MESSAGE_3 XCP_MEM_BLOCK_3_SIZE_is_not_large_enough_to_hold_a_pointer
CA_STATIC_ASSERT((XCP_ALIGNED(XCP_MEM_BLOCK_3_SIZE) >= XCP_MEM_SIZEOF_POINTER), XCP_MEM_ERROR_MESSAGE_3);

#endif

/* Block set 4 */
#if (defined(XCP_MEM_BLOCK_4_SIZE) ^ defined(XCP_MEM_BLOCK_4_NUMBER))
#error Error setting XCP_MEM_BLOCK_4_SIZE and XCP_MEM_BLOCK_4_NUMBER
#endif

#if (!defined(XCP_MEM_BLOCK_4_SIZE) & !defined(XCP_MEM_BLOCK_4_NUMBER)) || (XCP_MEM_BLOCK_4_SIZE == 0)
#define XCP_MEM_BLOCK_4_SIZE    0
#define XCP_MEM_BLOCK_4_NUMBER  0
#else

/* If we get here, both macros for set 4 are defined*/
#if ( XCP_MEM_BLOCK_3_NUMBER == 0 )
#error Error: XCP_MEM_BLOCK_4_SIZE and XCP_MEM_BLOCK_4_NUMBER are set but not XCP_MEM_BLOCK_3_SIZE and XCP_MEM_BLOCK_3_NUMBER
#endif

/* Check the condition that XCP_ALIGNED(XCP_MEM_BLOCK_4_SIZE) is large enough to hold a pointer */
#define XCP_MEM_ERROR_MESSAGE_4 XCP_MEM_BLOCK_4_SIZE_is_not_large_enough_to_hold_a_pointer
CA_STATIC_ASSERT((XCP_ALIGNED(XCP_MEM_BLOCK_4_SIZE) >= XCP_MEM_SIZEOF_POINTER), XCP_MEM_ERROR_MESSAGE_4);

#endif

/* Block set 5 */
#if (defined(XCP_MEM_BLOCK_5_SIZE) ^ defined(XCP_MEM_BLOCK_5_NUMBER))
#error Error setting XCP_MEM_BLOCK_5_SIZE and XCP_MEM_BLOCK_5_NUMBER
#endif

#if (!defined(XCP_MEM_BLOCK_5_SIZE) & !defined(XCP_MEM_BLOCK_5_NUMBER)) || (XCP_MEM_BLOCK_5_SIZE == 0)
#define XCP_MEM_BLOCK_5_SIZE    0
#define XCP_MEM_BLOCK_5_NUMBER  0
#else

/* If we get here, both macros for set 5 are defined*/
#if ( XCP_MEM_BLOCK_4_NUMBER == 0 )
#error Error: XCP_MEM_BLOCK_5_SIZE and XCP_MEM_BLOCK_5_NUMBER are set but not XCP_MEM_BLOCK_4_SIZE and XCP_MEM_BLOCK_4_NUMBER
#endif

/* Check the condition that XCP_ALIGNED(XCP_MEM_BLOCK_5_SIZE) is large enough to hold a pointer */
#define XCP_MEM_ERROR_MESSAGE_5 XCP_MEM_BLOCK_5_SIZE_is_not_large_enough_to_hold_a_pointer
CA_STATIC_ASSERT((XCP_ALIGNED(XCP_MEM_BLOCK_5_SIZE) >= XCP_MEM_SIZEOF_POINTER), XCP_MEM_ERROR_MESSAGE_5);

#endif

/* Block set 6 */
#if (defined(XCP_MEM_BLOCK_6_SIZE) ^ defined(XCP_MEM_BLOCK_6_NUMBER))
#error Error setting XCP_MEM_BLOCK_6_SIZE and XCP_MEM_BLOCK_6_NUMBER
#endif

#if (!defined(XCP_MEM_BLOCK_6_SIZE) & !defined(XCP_MEM_BLOCK_6_NUMBER)) || (XCP_MEM_BLOCK_6_SIZE == 0)
#define XCP_MEM_BLOCK_6_SIZE    0
#define XCP_MEM_BLOCK_6_NUMBER  0
#else

/* If we get here, both macros for set 6 are defined*/
#if ( XCP_MEM_BLOCK_5_NUMBER == 0 )
#error Error: XCP_MEM_BLOCK_6_SIZE and XCP_MEM_BLOCK_6_NUMBER are set but not XCP_MEM_BLOCK_5_SIZE and XCP_MEM_BLOCK_5_NUMBER
#endif

/* Check the condition that XCP_ALIGNED(XCP_MEM_BLOCK_6_SIZE) is large enough to hold a pointer */
#define XCP_MEM_ERROR_MESSAGE_6 XCP_MEM_BLOCK_6_SIZE_is_not_large_enough_to_hold_a_pointer
CA_STATIC_ASSERT((XCP_ALIGNED(XCP_MEM_BLOCK_6_SIZE) >= XCP_MEM_SIZEOF_POINTER), XCP_MEM_ERROR_MESSAGE_6);

#endif

/* Block set 7 */
#if (defined(XCP_MEM_BLOCK_7_SIZE) ^ defined(XCP_MEM_BLOCK_7_NUMBER))
#error Error setting XCP_MEM_BLOCK_7_SIZE and XCP_MEM_BLOCK_7_NUMBER
#endif

#if (!defined(XCP_MEM_BLOCK_7_SIZE) & !defined(XCP_MEM_BLOCK_7_NUMBER)) || (XCP_MEM_BLOCK_7_SIZE == 0)
#define XCP_MEM_BLOCK_7_SIZE    0
#define XCP_MEM_BLOCK_7_NUMBER  0
#else

/* If we get here, both macros for set 7 are defined*/
#if ( XCP_MEM_BLOCK_6_NUMBER == 0 )
#error Error: XCP_MEM_BLOCK_7_SIZE and XCP_MEM_BLOCK_7_NUMBER are set but not XCP_MEM_BLOCK_6_SIZE and XCP_MEM_BLOCK_6_NUMBER
#endif

/* Check the condition that XCP_ALIGNED(XCP_MEM_BLOCK_7_SIZE) is large enough to hold a pointer */
#define XCP_MEM_ERROR_MESSAGE_7 XCP_MEM_BLOCK_7_SIZE_is_not_large_enough_to_hold_a_pointer
CA_STATIC_ASSERT((XCP_ALIGNED(XCP_MEM_BLOCK_7_SIZE) >= XCP_MEM_SIZEOF_POINTER), XCP_MEM_ERROR_MESSAGE_7);

#endif


/* Block set 8 */
#if (defined(XCP_MEM_BLOCK_8_SIZE) ^ defined(XCP_MEM_BLOCK_8_NUMBER))
#error Error setting XCP_MEM_BLOCK_8_SIZE and XCP_MEM_BLOCK_8_NUMBER
#endif

#if (!defined(XCP_MEM_BLOCK_8_SIZE) & !defined(XCP_MEM_BLOCK_8_NUMBER)) || (XCP_MEM_BLOCK_8_SIZE == 0)
#define XCP_MEM_BLOCK_8_SIZE    0
#define XCP_MEM_BLOCK_8_NUMBER  0
#else

/* If we get here, both macros for set 8 are defined*/
#if ( XCP_MEM_BLOCK_7_NUMBER == 0 )
#error Error: XCP_MEM_BLOCK_8_SIZE and XCP_MEM_BLOCK_8_NUMBER are set but not XCP_MEM_BLOCK_7_SIZE and XCP_MEM_BLOCK_7_NUMBER
#endif

/* Check the condition that XCP_ALIGNED(XCP_MEM_BLOCK_8_SIZE) is large enough to hold a pointer */
#define XCP_MEM_ERROR_MESSAGE_8 XCP_MEM_BLOCK_8_SIZE_is_not_large_enough_to_hold_a_pointer
CA_STATIC_ASSERT((XCP_ALIGNED(XCP_MEM_BLOCK_8_SIZE) >= XCP_MEM_SIZEOF_POINTER), XCP_MEM_ERROR_MESSAGE_8);

#endif


/* Block set 9 */
#if (defined(XCP_MEM_BLOCK_9_SIZE) ^ defined(XCP_MEM_BLOCK_9_NUMBER))
#error Error setting XCP_MEM_BLOCK_9_SIZE and XCP_MEM_BLOCK_9_NUMBER
#endif

#if (!defined(XCP_MEM_BLOCK_9_SIZE) & !defined(XCP_MEM_BLOCK_9_NUMBER)) || (XCP_MEM_BLOCK_9_SIZE == 0)
#define XCP_MEM_BLOCK_9_SIZE    0
#define XCP_MEM_BLOCK_9_NUMBER  0
#else

/* If we get here, both macros for set 9 are defined*/
#if ( XCP_MEM_BLOCK_8_NUMBER == 0 )
#error Error: XCP_MEM_BLOCK_9_SIZE and XCP_MEM_BLOCK_9_NUMBER are set but not XCP_MEM_BLOCK_8_SIZE and XCP_MEM_BLOCK_8_NUMBER
#endif

/* Check the condition that XCP_ALIGNED(XCP_MEM_BLOCK_9_SIZE) is large enough to hold a pointer */
#define XCP_MEM_ERROR_MESSAGE_9 XCP_MEM_BLOCK_9_SIZE_is_not_large_enough_to_hold_a_pointer
CA_STATIC_ASSERT((XCP_ALIGNED(XCP_MEM_BLOCK_9_SIZE) >= XCP_MEM_SIZEOF_POINTER), XCP_MEM_ERROR_MESSAGE_9);

#endif


/* Block set 10 */
#if (defined(XCP_MEM_BLOCK_10_SIZE) ^ defined(XCP_MEM_BLOCK_10_NUMBER))
#error Error setting XCP_MEM_BLOCK_10_SIZE and XCP_MEM_BLOCK_10_NUMBER
#endif

#if (!defined(XCP_MEM_BLOCK_10_SIZE) & !defined(XCP_MEM_BLOCK_10_NUMBER)) || (XCP_MEM_BLOCK_10_SIZE == 0)
#define XCP_MEM_BLOCK_10_SIZE    0
#define XCP_MEM_BLOCK_10_NUMBER  0
#else

/* If we get here, both macros for set 10 are defined*/
#if ( XCP_MEM_BLOCK_9_NUMBER == 0 )
#error Error: XCP_MEM_BLOCK_10_SIZE and XCP_MEM_BLOCK_10_NUMBER are set but not XCP_MEM_BLOCK_9_SIZE and XCP_MEM_BLOCK_9_NUMBER
#endif

/* Check the condition that XCP_ALIGNED(XCP_MEM_BLOCK_10_SIZE) is large enough to hold a pointer */
#define XCP_MEM_ERROR_MESSAGE_10 XCP_MEM_BLOCK_10_SIZE_is_not_large_enough_to_hold_a_pointer
CA_STATIC_ASSERT((XCP_ALIGNED(XCP_MEM_BLOCK_10_SIZE) >= XCP_MEM_SIZEOF_POINTER), XCP_MEM_ERROR_MESSAGE_10);

#endif

/* Block set 11 */
#if (defined(XCP_MEM_BLOCK_11_SIZE) ^ defined(XCP_MEM_BLOCK_11_NUMBER))
#error Error setting XCP_MEM_BLOCK_11_SIZE and XCP_MEM_BLOCK_11_NUMBER
#endif

#if (!defined(XCP_MEM_BLOCK_11_SIZE) & !defined(XCP_MEM_BLOCK_11_NUMBER)) || (XCP_MEM_BLOCK_11_SIZE == 0)
#define XCP_MEM_BLOCK_11_SIZE    0
#define XCP_MEM_BLOCK_11_NUMBER  0
#else

/* If we get here, both macros for set 11 are defined*/
#if ( XCP_MEM_BLOCK_10_NUMBER == 0 )
#error Error: XCP_MEM_BLOCK_11_SIZE and XCP_MEM_BLOCK_11_NUMBER are set but not XCP_MEM_BLOCK_10_SIZE and XCP_MEM_BLOCK_10_NUMBER
#endif

/* Check the condition that XCP_ALIGNED(XCP_MEM_BLOCK_11_SIZE) is large enough to hold a pointer */
#define XCP_MEM_ERROR_MESSAGE_11 XCP_MEM_BLOCK_11_SIZE_is_not_large_enough_to_hold_a_pointer
CA_STATIC_ASSERT((XCP_ALIGNED(XCP_MEM_BLOCK_11_SIZE) >= XCP_MEM_SIZEOF_POINTER), XCP_MEM_ERROR_MESSAGE_11);

#endif

/* Block set 12 */
#if (defined(XCP_MEM_BLOCK_12_SIZE) ^ defined(XCP_MEM_BLOCK_12_NUMBER))
#error Error setting XCP_MEM_BLOCK_12_SIZE and XCP_MEM_BLOCK_12_NUMBER
#endif

#if (!defined(XCP_MEM_BLOCK_12_SIZE) & !defined(XCP_MEM_BLOCK_12_NUMBER)) || (XCP_MEM_BLOCK_12_SIZE == 0)
#define XCP_MEM_BLOCK_12_SIZE    0
#define XCP_MEM_BLOCK_12_NUMBER  0
#else

/* If we get here, both macros for set 12 are defined*/
#if ( XCP_MEM_BLOCK_11_NUMBER == 0 )
#error Error: XCP_MEM_BLOCK_12_SIZE and XCP_MEM_BLOCK_12_NUMBER are set but not XCP_MEM_BLOCK_11_SIZE and XCP_MEM_BLOCK_11_NUMBER
#endif

/* Check the condition that XCP_ALIGNED(XCP_MEM_BLOCK_12_SIZE) is large enough to hold a pointer */
#define XCP_MEM_ERROR_MESSAGE_12 XCP_MEM_BLOCK_12_SIZE_is_not_large_enough_to_hold_a_pointer
CA_STATIC_ASSERT((XCP_ALIGNED(XCP_MEM_BLOCK_12_SIZE) >= XCP_MEM_SIZEOF_POINTER), XCP_MEM_ERROR_MESSAGE_12);

#endif

/* Block set 13 */
#if (defined(XCP_MEM_BLOCK_13_SIZE) ^ defined(XCP_MEM_BLOCK_13_NUMBER))
#error Error setting XCP_MEM_BLOCK_13_SIZE and XCP_MEM_BLOCK_13_NUMBER
#endif

#if (!defined(XCP_MEM_BLOCK_13_SIZE) & !defined(XCP_MEM_BLOCK_13_NUMBER)) || (XCP_MEM_BLOCK_13_SIZE == 0)
#define XCP_MEM_BLOCK_13_SIZE    0
#define XCP_MEM_BLOCK_13_NUMBER  0
#else

/* If we get here, both macros for set 13 are defined*/
#if ( XCP_MEM_BLOCK_12_NUMBER == 0 )
#error Error: XCP_MEM_BLOCK_13_SIZE and XCP_MEM_BLOCK_13_NUMBER are set but not XCP_MEM_BLOCK_12_SIZE and XCP_MEM_BLOCK_12_NUMBER
#endif

/* Check the condition that XCP_ALIGNED(XCP_MEM_BLOCK_13_SIZE) is large enough to hold a pointer */
#define XCP_MEM_ERROR_MESSAGE_13 XCP_MEM_BLOCK_13_SIZE_is_not_large_enough_to_hold_a_pointer
CA_STATIC_ASSERT((XCP_ALIGNED(XCP_MEM_BLOCK_13_SIZE) >= XCP_MEM_SIZEOF_POINTER), XCP_MEM_ERROR_MESSAGE_13);

#endif

/* Block set 14 */
#if (defined(XCP_MEM_BLOCK_14_SIZE) ^ defined(XCP_MEM_BLOCK_14_NUMBER))
#error Error setting XCP_MEM_BLOCK_14_SIZE and XCP_MEM_BLOCK_14_NUMBER
#endif

#if (!defined(XCP_MEM_BLOCK_14_SIZE) & !defined(XCP_MEM_BLOCK_14_NUMBER)) || (XCP_MEM_BLOCK_14_SIZE == 0)
#define XCP_MEM_BLOCK_14_SIZE    0
#define XCP_MEM_BLOCK_14_NUMBER  0
#else

/* If we get here, both macros for set 14 are defined*/
#if ( XCP_MEM_BLOCK_13_NUMBER == 0 )
#error Error: XCP_MEM_BLOCK_14_SIZE and XCP_MEM_BLOCK_14_NUMBER are set but not XCP_MEM_BLOCK_13_SIZE and XCP_MEM_BLOCK_13_NUMBER
#endif

/* Check the condition that XCP_ALIGNED(XCP_MEM_BLOCK_14_SIZE) is large enough to hold a pointer */
#define XCP_MEM_ERROR_MESSAGE_14 XCP_MEM_BLOCK_14_SIZE_is_not_large_enough_to_hold_a_pointer
CA_STATIC_ASSERT((XCP_ALIGNED(XCP_MEM_BLOCK_14_SIZE) >= XCP_MEM_SIZEOF_POINTER), XCP_MEM_ERROR_MESSAGE_14);

#endif

/* Block set 15 */
#if (defined(XCP_MEM_BLOCK_15_SIZE) ^ defined(XCP_MEM_BLOCK_15_NUMBER))
#error Error setting XCP_MEM_BLOCK_15_SIZE and XCP_MEM_BLOCK_15_NUMBER
#endif

#if (!defined(XCP_MEM_BLOCK_15_SIZE) & !defined(XCP_MEM_BLOCK_15_NUMBER)) || (XCP_MEM_BLOCK_15_SIZE == 0)
#define XCP_MEM_BLOCK_15_SIZE    0
#define XCP_MEM_BLOCK_15_NUMBER  0
#else

/* If we get here, both macros for set 15 are defined*/
#if ( XCP_MEM_BLOCK_14_NUMBER == 0 )
#error Error: XCP_MEM_BLOCK_15_SIZE and XCP_MEM_BLOCK_15_NUMBER are set but not XCP_MEM_BLOCK_14_SIZE and XCP_MEM_BLOCK_14_NUMBER
#endif

/* Check the condition that XCP_ALIGNED(XCP_MEM_BLOCK_12_SIZE) is large enough to hold a pointer */
#define XCP_MEM_ERROR_MESSAGE_15 XCP_MEM_BLOCK_15_SIZE_is_not_large_enough_to_hold_a_pointer
CA_STATIC_ASSERT((XCP_ALIGNED(XCP_MEM_BLOCK_15_SIZE) >= XCP_MEM_SIZEOF_POINTER), XCP_MEM_ERROR_MESSAGE_15);

#endif

/* Block set 16 */
#if (defined(XCP_MEM_BLOCK_16_SIZE) ^ defined(XCP_MEM_BLOCK_16_NUMBER))
#error Error setting XCP_MEM_BLOCK_16_SIZE and XCP_MEM_BLOCK_16_NUMBER
#endif

#if (!defined(XCP_MEM_BLOCK_16_SIZE) & !defined(XCP_MEM_BLOCK_16_NUMBER)) || (XCP_MEM_BLOCK_16_SIZE == 0)
#define XCP_MEM_BLOCK_16_SIZE    0
#define XCP_MEM_BLOCK_16_NUMBER  0
#else

/* If we get here, both macros for set 16 are defined*/
#if ( XCP_MEM_BLOCK_15_NUMBER == 0 )
#error Error: XCP_MEM_BLOCK_16_SIZE and XCP_MEM_BLOCK_16_NUMBER are set but not XCP_MEM_BLOCK_15_SIZE and XCP_MEM_BLOCK_15_NUMBER
#endif

/* Check the condition that XCP_ALIGNED(XCP_MEM_BLOCK_12_SIZE) is large enough to hold a pointer */
#define XCP_MEM_ERROR_MESSAGE_16 XCP_MEM_BLOCK_16_SIZE_is_not_large_enough_to_hold_a_pointer
CA_STATIC_ASSERT((XCP_ALIGNED(XCP_MEM_BLOCK_16_SIZE) >= XCP_MEM_SIZEOF_POINTER), XCP_MEM_ERROR_MESSAGE_16);

#endif

#endif /* XCP_MEM_CONFIG_HELPER_H */
