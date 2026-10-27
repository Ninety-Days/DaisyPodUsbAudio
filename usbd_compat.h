/**
 * usbd_compat.h — build against both pre- and post-composite versions of
 * ST's STM32_USB_Device_Library, without a version macro.
 *
 * MIT License (see LICENSE at repository root) applies to this file.
 * It contains no ST code; it only accesses fields of ST's
 * USBD_HandleTypeDef. See THIRD_PARTY_NOTICES.md.
 *
 * ---------------------------------------------------------------------
 * THE PROBLEM
 *
 * ST introduced composite-device support in the USB Device Library, and
 * that one change altered the *type* of two fields in USBD_HandleTypeDef:
 *
 *              pre-composite                post-composite
 *   pClass     USBD_ClassTypeDef *pClass;   USBD_ClassTypeDef *pClass[N];
 *   pUserData  void *pUserData;             void *pUserData[N];
 *
 *   (N == USBD_MAX_SUPPORTED_CLASS. pClassData stayed a plain scalar in
 *   both; pClassDataCmsit[] was added alongside it, not in place of it,
 *   so every pdev->pClassData site in this tree is unaffected.)
 *
 * Consequences of upgrading libDaisy across that boundary, both real:
 *
 *   - `pdev->pUserData = fops;` becomes a hard error --
 *     "assignment to expression with array type".
 *   - `pdev->pClass->Setup(...)` becomes a hard error -- `->` applied to
 *     an array.
 *
 * And one that is *not* an error, which is the dangerous one:
 *
 *   - `(USBD_AUDIO_ItfTypeDef *)pdev->pUserData` still compiles on the
 *     array form. The array decays to void**, and casting void** to a
 *     struct pointer is legal C with no diagnostic. It would silently
 *     yield a pointer to the array's own storage rather than to the fops
 *     that were stored in it. A compiler cannot catch this one; only
 *     going through the accessor below prevents it.
 *
 * ---------------------------------------------------------------------
 * WHY _Generic AND NOT #ifdef
 *
 * The obvious move is `#ifdef USBD_MAX_SUPPORTED_CLASS`. It does not
 * work. That macro is defined *twice* inside the post-composite
 * usbd_def.h (4U under one #ifndef, 1U under another) and its presence
 * says nothing reliable about which tree is being compiled -- testing it
 * is a guess dressed up as a check. classId has the same problem: it is
 * a struct field, and #ifdef cannot see struct fields at all.
 *
 * _Generic does not guess. It dispatches on the field's actual type as
 * the compiler sees it, so it is correct by construction on any tree,
 * including future ones that keep either shape. Requires C11; this
 * project builds with -std=gnu11 (from libDaisy/core/Makefile), and the
 * static assertion at the bottom of this file fails the build loudly if
 * that ever stops being true.
 *
 * NOTE: _Generic is C-only. It does not exist in C++. Every file that
 * includes this header must be compiled as C -- in this tree that is all
 * of C_SOURCES. main.cpp is C++ and must not use these macros; it does
 * not need to, since it only calls USBD_RegisterClass().
 *
 * ---------------------------------------------------------------------
 * INDEX 0
 *
 * On the array form both accessors use slot 0, not pdev->classId. This
 * device registers exactly one class (USBD_RegisterClass in main.cpp,
 * called once), so slot 0 is the only slot ever populated and classId is
 * 0 throughout. If this tree ever registers a second class, these macros
 * are wrong and must take the index explicitly -- that is a deliberate
 * limit, not an oversight.
 */
#ifndef __USBD_COMPAT_H
#define __USBD_COMPAT_H

#ifdef __cplusplus
#error "usbd_compat.h uses _Generic and is C-only; do not include it from C++."
#endif

#include "usbd_def.h"

/* Address of the cell holding this class's fops pointer.
 * Use as an lvalue for both read and write:
 *     *USBD_COMPAT_USERDATA_SLOT(pdev) = fops;
 *     ItfTypeDef *ops = *USBD_COMPAT_USERDATA_SLOT(pdev); */
#define USBD_COMPAT_USERDATA_SLOT(p) _Generic((p)->pUserData, \
    void *  : (void **)&(p)->pUserData,                       \
    default : (void **)(p)->pUserData)

/* The active USBD_ClassTypeDef, as a plain pointer on both shapes.
 *     USBD_COMPAT_CLASS(pdev)->Setup(pdev, req); */
#define USBD_COMPAT_CLASS(p) _Generic((p)->pClass,                    \
    USBD_ClassTypeDef * : (USBD_ClassTypeDef *)(p)->pClass,           \
    default             : ((USBD_ClassTypeDef **)(p)->pClass)[0])

/* ---------------------------------------------------------------------
 * USBD_LL_XFER_SIZE_T — 4th parameter of USBD_LL_Transmit and
 * USBD_LL_PrepareReceive. The same ST modernization that made pClass and
 * pUserData arrays also widened this parameter from uint16_t to uint32_t.
 *
 * _Generic CANNOT solve this one. It selects between expressions, but
 * what we need is the parameter type in a function *definition*, which
 * only the preprocessor can choose. So this is a guess -- and unlike the
 * two accessors above, it is not correct by construction.
 *
 * What makes it safe is the companion _Static_assert in usbd_conf.c: it
 * compares this type against the prototype ST actually declared and
 * fails the build with a one-line instruction if the guess is wrong.
 * A wrong guess therefore costs one -D flag, never a silent mismatch.
 *
 * The default keys off USBD_MAX_SUPPORTED_CLASS, which the composite
 * rework introduced. That correlation has been verified on the new tree
 * only; whether every pre-composite usbd_def.h truly lacks the macro has
 * NOT been confirmed against an old checkout. If it turns out some old
 * tree defines it, the assert fires and you override:
 *
 *     make C_DEFS+=-DUSBD_LL_XFER_SIZE_T=uint16_t
 */
#ifndef USBD_LL_XFER_SIZE_T
#  ifdef USBD_MAX_SUPPORTED_CLASS
#    define USBD_LL_XFER_SIZE_T uint32_t
#  else
#    define USBD_LL_XFER_SIZE_T uint16_t
#  endif
#endif

/* ---------------------------------------------------------------------
 * Reject a shape this header does not know.
 *
 * The two accessors above are correct for the two shapes ST has shipped.
 * A third would not be caught by either: `default` accepts anything, and
 * casting some other pointer type to void** is legal C with no
 * diagnostic, so the macros would compile and hand back a pointer into
 * the wrong storage. That is a silent failure, and silent is the one
 * thing this header must not be.
 *
 * The probe is declared and never defined. _Generic does not evaluate
 * its controlling expression, so no symbol is emitted and nothing has to
 * exist at link time.
 */
extern USBD_HandleTypeDef *usbd_compat_shape_probe;

_Static_assert(_Generic(usbd_compat_shape_probe->pUserData,
        void *  : 1,
        void ** : 1,
        default : 0),
    "usbd_compat.h: USBD_HandleTypeDef::pUserData is neither void* nor "
    "void*[]. This ST middleware uses a field shape this header does not "
    "know, and the accessors would silently return a wrong pointer. Update "
    "usbd_compat.h rather than deleting this assertion.");

_Static_assert(_Generic(usbd_compat_shape_probe->pClass,
        USBD_ClassTypeDef *  : 1,
        USBD_ClassTypeDef ** : 1,
        default              : 0),
    "usbd_compat.h: USBD_HandleTypeDef::pClass is neither USBD_ClassTypeDef* "
    "nor USBD_ClassTypeDef*[]. Same situation as pUserData above; update this "
    "header rather than deleting the assertion.");

/* Fail the build with a readable message if _Generic is unavailable,
 * rather than emitting a wall of syntax errors from the macros above. */
#if !defined(__STDC_VERSION__) || (__STDC_VERSION__ < 201112L)
#error "usbd_compat.h requires C11 (_Generic). Build with -std=gnu11 or newer."
#endif

#endif /* __USBD_COMPAT_H */
