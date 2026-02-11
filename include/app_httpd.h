#pragma once

typedef void (*loopCallback_t)();

void InitHttpd();
void LoopHttpd(loopCallback_t);

extern bool isStreaming;