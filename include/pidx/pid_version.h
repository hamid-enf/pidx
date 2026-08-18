/**
 * @file    pid_version.h
 * @brief   PIDX library version and ABI compatibility helpers.
 *
 * Semantic versioning:
 *   MAJOR - breaking API/ABI change
 *   MINOR - backward-compatible feature addition
 *   PATCH - backward-compatible fix
 *
 * The Level-1 (Basic) API is frozen at MAJOR 1 and will not change.
 */
#ifndef PIDX_PID_VERSION_H
#define PIDX_PID_VERSION_H

#define PIDX_VERSION_MAJOR   1
#define PIDX_VERSION_MINOR   0
#define PIDX_VERSION_PATCH   0

#define PIDX_VERSION_NUM \
    (((PIDX_VERSION_MAJOR) * 10000) + ((PIDX_VERSION_MINOR) * 100) + (PIDX_VERSION_PATCH))

#define PIDX_VERSION_STRING  "1.0.0"

/**
 * ABI version stamped into PID_Config by PID_ConfigDefault().
 * PID_Init() rejects a config whose stamp it does not understand, which makes
 * "user recompiled the app but linked an old library" fail loudly instead of
 * silently reading garbage out of newly added struct fields.
 */
#define PIDX_CONFIG_ABI_VERSION  1u

#endif /* PIDX_PID_VERSION_H */
