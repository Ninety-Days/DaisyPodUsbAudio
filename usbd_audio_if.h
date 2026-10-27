/**
 * usbd_audio_if.h — declarations for the application-level audio/MIDI
 * interface implemented in usbd_audio_if.c.
 *
 * MIT License (see LICENSE at repository root) applies to this file.
 * THIRD_PARTY_NOTICES.md carries the repository's file-by-file breakdown.
 */
#ifndef USBD_AUDIO_IF_H
#define USBD_AUDIO_IF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_audio.h"

extern USBD_AUDIO_ItfTypeDef USBD_AUDIO_fops;

/* Record path (ADC → host). Call every AudioCallback with the interleaved
 * L/R block: int32_t holding sign-extended 24-bit values, +-8388607.
 * main.cpp scales its native +-1.0f float by 8388608.0f (2^23), not by
 * 32768.0f. ISR-safe; frames that overflow the ring are dropped. */
void AudioIF_PushSamples(int32_t *samples, uint32_t num_frames);

/* Playback path (host → DAC), same sample format as above. A short read
 * is zero-filled and counted in pb_underrun_events. ISR-safe. */
void AudioIF_PopSamples(int32_t *dst, uint32_t num_frames);

/* Resets both rings and the MIDI CC table, so no stale audio or phantom
 * CC value survives a reconnect.
 *
 * SCOPE WARNING: because it resets everything, call it only from class
 * init (Audio_Init), never from a per-interface SET_INTERFACE branch.
 * Doing the latter is what let a MIC-interface event destroy a running
 * playback stream; use the per-path functions below instead. */
void AudioIF_Reset(void);

/* Per-path ring reset, split out of AudioIF_Reset. The combined reset ran
 * from all four SET_INTERFACE branches, so any mic-side alt-setting change
 * — which the host issues independently of the speaker stream — wiped the
 * playback ring mid-stream and cost ~5.33 ms of silence while it re-primed.
 *
 * These do not touch dsp_params: a CC value has nothing to do with an
 * audio alt-setting. */
void AudioIF_ResetRecord(void);
void AudioIF_ResetPlayback(void);

/* Mirrors out_streaming here. Without it AudioIF_PopSamples would
 * circulate stale frames from an earlier activation instead of silence
 * when the host is not driving the stream. */
void AudioIF_SetPlaybackActive(uint8_t active);

/* Read-only accessors. They let callers outside this file — the LED
 * diagnostics in main.cpp — tell "host still has the stream open" from
 * "gate is closed but samples still leak through". */
uint8_t AudioIF_IsPlaybackActive(void);
uint8_t AudioIF_IsRecordActive(void);


/* MIDI path. Control Change messages on Bulk OUT 0x02 are stored in
 * dsp_params[cc_number] and nothing more — this firmware never acts on
 * the values. Parses 4-byte USB MIDI event packets; only CIN=0x0B updates
 * the table, everything else is ignored. ISR-safe. */
void AudioIF_PushMIDI(uint8_t *buf, uint32_t num_bytes);

/* Called from the DMA ISR. Single-byte volatile read, atomic on Cortex-M7,
 * so no mutex. Returns 0 for a CC never received. */
uint8_t AudioIF_GetCCParam(uint8_t cc_number);

/* Diagnostics, record path. up/down tick while the corrector pulls the
 * read rate above or below nominal; rec_underrun_events counts the ring
 * hitting zero.
 *
 * These say how hard the corrector is pulling, NOT how many
 * discontinuities were emitted — the fractional reader emits none, and the
 * packet is always exactly nominal length.
 *
 * Healthy: up and down both non-zero, rec_underrun = 0. */
extern volatile uint32_t drift_corrections_up;
extern volatile uint32_t drift_corrections_down;
extern volatile uint32_t rec_underrun_events;

extern volatile uint32_t pb_overrun_events;
extern volatile uint32_t pb_underrun_events;
extern volatile uint32_t pb_read_advances;
extern volatile uint32_t pb_max_avail_seen;

/* -----------------------------------------------------------------------
 * Diagnostics — split rec/pb drift attribution
 *
 * drift_corrections_up/down (above) are the SUM of both controllers.
 * These four split the same events out by which side actually fired
 * them, so a host-side script can tell rec-side oscillation apart from
 * pb-side oscillation instead of only seeing a combined total. */
extern volatile uint32_t rec_drift_corrections_up;
extern volatile uint32_t rec_drift_corrections_down;
extern volatile uint32_t pb_drift_corrections_up;
extern volatile uint32_t pb_drift_corrections_down;

/* Min/max "available" frames observed on each ring (rec_max_avail_seen
 * is new; pb_max_avail_seen already existed). Plus the single most
 * recent snapshot on each side, for live polling between extremes. */
extern volatile uint32_t rec_max_avail_seen;
extern volatile uint32_t rec_min_avail_seen;
extern volatile uint32_t pb_min_avail_seen;
extern volatile uint32_t rec_last_avail;
extern volatile uint32_t pb_last_avail;

/* CRS (Clock Recovery System) sync-event counters -- see usbd_audio_if.c
 * for why these are cumulative counts rather than raw ISR flags, and why
 * Audio_PeriodicTC (not the USB control-request handler) owns clearing
 * the underlying CRS->ISR bits. */
extern volatile uint32_t crs_sync_ok_count;
extern volatile uint32_t crs_sync_warn_count;
extern volatile uint32_t crs_sync_miss_count;
extern volatile uint32_t crs_trim_ovf_count;

/* Deeper CRS diagnostics -- see the block comment above these two
 * variables' definitions in usbd_audio_if.c for why they were added
 * (crs_sync_ok_count alone couldn't distinguish "SYNCOK never fires"
 * from "the polling loop somehow isn't running"). */
extern volatile uint32_t crs_trim_snapshot;
extern volatile uint32_t crs_poll_checked_count;

/* Current fractional-corrector rate tilt, Q16 (step_q16 - FRAC_ONE),
 * range +-32 where 32 == 488 ppm. The state itself stays static to
 * usbd_audio_if.c, each half touched only from its owning ISR; these
 * getters are the supported way to observe it from the vendor-request
 * handler in usbd_audio.c. A single int32_t read is atomic on Cortex-M7,
 * so no locking is needed. Vendor diagnostic slots 10/11 carry these. */
int32_t AudioIF_GetRecTiltQ16(void);
int32_t AudioIF_GetPbTiltQ16(void);

/* Volume and mute, SPK Feature Unit (ID=5), driven by the SET_CUR/GET_CUR
 * handlers in usbd_audio.c.
 *
 * All of it is applied in software, in AudioIF_PopSamples: the PCM3060 on
 * this board is hardware-mode-strapped and no I2C attenuation register is
 * listening. The same is true of the mic side below.
 *
 * raw_db_q8 is UAC1's native signed 1/256 dB format, the same encoding the
 * wire uses, so GET_CUR echoes back exactly what was set. Mute is a
 * separate boolean multiplied on top of the gain, not folded into the dB
 * value. Range is 0 dB .. -96 dB throughout. */
void    AudioIF_SetVolumeRaw(int16_t raw_db_q8);
int16_t AudioIF_GetVolumeRaw(void);
void    AudioIF_SetMuteState(uint8_t muted);
uint8_t AudioIF_GetMuteState(void);

/* Per-channel SPK volume and mute, independent of the master above.
 * Final gain is master * channel, mute is master || channel, as most UAC1
 * devices combine them. */
void    AudioIF_SetVolumeRawL(int16_t raw_db_q8);
int16_t AudioIF_GetVolumeRawL(void);
void    AudioIF_SetMuteStateL(uint8_t muted);
uint8_t AudioIF_GetMuteStateL(void);

void    AudioIF_SetVolumeRawR(int16_t raw_db_q8);
int16_t AudioIF_GetVolumeRawR(void);
void    AudioIF_SetMuteStateR(uint8_t muted);
uint8_t AudioIF_GetMuteStateR(void);

/* MIC-side counterparts, independent state, same encoding. */
void    AudioIF_SetVolumeRawMic(int16_t raw_db_q8);
int16_t AudioIF_GetVolumeRawMic(void);
void    AudioIF_SetMuteStateMic(uint8_t muted);
uint8_t AudioIF_GetMuteStateMic(void);

#ifdef __cplusplus
}
#endif

#endif /* USBD_AUDIO_IF_H */
