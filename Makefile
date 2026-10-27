TARGET = DaisyPodUsbAudio

CPP_SOURCES = main.cpp

C_SOURCES = usbd_audio.c \
            usbd_audio_if.c \
            usbd_conf.c \
            usbd_desc.c \
            usbd_irq.c \
            usbd_ctlreq.c

LIBDAISY_DIR = ../../libDaisy

# DAISYSP_DIR is deliberately not set. libDaisy's core/Makefile adds
# -ldaisysp and its include path only `ifdef DAISYSP_DIR`, so leaving it
# undefined drops the dependency entirely. This firmware has no processing
# stage and calls nothing from DaisySP, so requiring it would only mean a
# second library to clone and build for no linked code.

# -Wl,--allow-multiple-definition is required. Removing it was tried and the
# link fails; what follows is what the linker actually said, not a guess.
#
# This project ships its own usbd_conf.c, which is a near-complete duplicate
# of libDaisy's src/usbd/usbd_conf.c. Both end up in the link and collide on
# roughly thirty symbols: Error_Handler, every HAL_PCD_*Callback, every
# USBD_LL_* entry point, and hpcd_USB_OTG_FS itself.
#
# A static-library member is only pulled in when it resolves an undefined
# symbol, and the map file names the one responsible:
#
#     libdaisy.a(usbd_conf.o)
#             libdaisy.a(system.o) (hpcd_USB_OTG_HS)
#
# daisy_seed.o references System::DeInit(), which lives in system.o, which
# references hpcd_USB_OTG_HS. That handle is defined only in libDaisy's
# usbd_conf.o -- this project's copy defines hpcd_USB_OTG_FS and nothing
# else, the HS layer having been removed deliberately (see usbd_desc.c).
# One missing symbol therefore drags in an object carrying thirty
# definitions we already have. libdaisy.a(usbh_conf.o) arrives the same way,
# via hhcd_USB_OTG_HS.
#
# Defining a dummy hpcd_USB_OTG_HS here would resolve that reference locally
# and keep libDaisy's usbd_conf.o out of the link entirely, which would make
# this flag unnecessary and save its dead code. Not done: whether system.o
# merely names the handle or actually calls into it has not been checked,
# and a zeroed PCD handle reached at runtime is worse than a linker warning.
# Check before trying it.
#
# Our objects precede -ldaisy on the link line, so GNU ld's "first
# definition wins" rule selects ours; the flag only downgrades the duplicate
# to a warning. The previous version of this comment blamed
# hw.seed.StartLog()/PrintLine() for pulling the conflict in. Neither is
# called anywhere in this tree, so that explanation was already stale when
# it was written down.
LDFLAGS += -Wl,--allow-multiple-definition

SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile
