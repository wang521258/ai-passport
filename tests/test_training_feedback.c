#include <assert.h>
#include <stdint.h>
#include "training_feedback.h"

int main(void)
{
    training_feedback_t f = {0};
    assert(!training_feedback_ready(&f, 5000, true));
    assert(training_feedback_begin(&f, true, 100));
    assert(!training_feedback_begin(&f, false, 120)); /* double click cannot rescore */
    assert(f.right && f.started == 100);
    assert(!training_feedback_ready(&f, 749, true));
    assert(training_feedback_ready(&f, 750, true));
    assert(!training_feedback_ready(&f, 1699, false));
    assert(training_feedback_ready(&f, 1700, false));
    training_feedback_clear(&f);
    assert(!training_feedback_ready(&f, 1800, true));
    assert(training_feedback_begin(&f, false, 2000));
    assert(!training_feedback_ready(&f, 900000, false)); /* wrong answer stays readable */
    assert(training_feedback_ready(&f, 2650, true));
    training_feedback_clear(&f); /* exiting a page cancels pending progression */
    assert(!training_feedback_ready(&f, 900000, false));
    assert(training_feedback_begin(&f, true, UINT32_MAX - 100));
    assert(!training_feedback_ready(&f, 548, true));
    assert(training_feedback_ready(&f, 549, true));
    assert(training_feedback_ready(&f, 1499, false));
    return 0;
}
