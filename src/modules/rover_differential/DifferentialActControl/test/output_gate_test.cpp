// SPDX-License-Identifier: BSD-3-Clause
#include "../DifferentialOutputGate.hpp"
#include <assert.h>
#include <stdio.h>

using namespace rover_differential;

int main()
{
	assert(mode_allows(false, false, false, 0, 500000));
	assert(!mode_allows(true, false, true, 1000, 1001));
	assert(!mode_allows(true, true, false, 1000, 1001));
	assert(mode_allows(true, true, true, 1000, 200999));
	assert(!mode_allows(true, true, true, 1000, 201000));
	assert(!mode_allows(true, true, true, 2000, 1000));
	OutputGate gate;
	SafetySnapshot safe{1000, SafetyStateSafe, true, true, true, true, false, false};
	assert(gate.allow(false, safe, 1000, 1000, 1000, true, true));
	assert(!gate.allow(true, safe, 1000, 1000, 1000, true, true));
	assert(gate.allow(true, safe, 2000, 2000, 2000, true, true));
	safe.stop_request = true;
	assert(!gate.allow(true, safe, 3000, 3000, 3000, true, true));
	safe.stop_request = false;
	safe.latched = false;
	safe.timestamp = 4000;
	assert(!gate.allow(true, safe, 4000, 4000, 4000, true, true));
	assert(gate.allow(true, safe, 5000, 5000, 5000, true, true));
	assert(!gate.allow(true, safe, 6000, 5000, 600000, false, true));
	assert(!gate.allow(true, safe, 600001, 600001, 600001, true, true));
	safe.timestamp = 600002;
	assert(!gate.allow(true, safe, 600002, 600002, 600002, true, true));
	assert(gate.allow(true, safe, 600003, 600003, 600003, true, true));
	safe.left_valid = false;
	assert(!gate.allow(true, safe, 600004, 600004, 600004, true, true));
	puts("PASS: output gate safety and freshness checks");
}
