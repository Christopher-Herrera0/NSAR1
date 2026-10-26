#pragma once

void cascade_reset();
void brain_log();
void intake_antijam();
extern volatile bool cascade_resetting;
extern volatile bool intake_jammed;

