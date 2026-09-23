#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool active;
    bool right;
    uint32_t started;
} training_feedback_t;

static inline bool training_feedback_begin(training_feedback_t *f, bool right, uint32_t now)
{
    if (f->active) return false;
    *f = (training_feedback_t){true, right, now};
    return true;
}

/* Unsigned subtraction also works across the LVGL tick counter wrap. */
static inline bool training_feedback_ready(const training_feedback_t *f, uint32_t now, bool confirm)
{
    uint32_t elapsed = now - f->started;
    return f->active && ((confirm && elapsed >= 650) || (f->right && elapsed >= 1600));
}

static inline void training_feedback_clear(training_feedback_t *f)
{
    f->active = false;
}
