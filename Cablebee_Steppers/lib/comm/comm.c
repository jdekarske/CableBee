#include <comm.h>

int parseSerialCommand(uint8_t bytes[], uint8_t length)
{
    uint8_t buffer_out[0];

    struct Packet packet_out;

    switch (buffer_out[0])
    {
    case MOVE:
        break;
    case CONFIG:
        break;
    case STOP:
        break;
    case ESTOP:
        break;
    default:
        return -1;
    }

    // throw that baby on the queueue
    packet_out.command = buffer_out[0];

    return 0;
}

void handleMove(int32_t steps[], int32_t speeds[])
{}
void handleConfig(uint8_t channel)
{}
void handleStop(uint8_t channel)
{}
void handleEStop(uint8_t channel)
{}
