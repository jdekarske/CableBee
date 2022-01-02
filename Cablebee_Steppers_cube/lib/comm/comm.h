/**
 * AUTHOR: Jason Dekarske (dekarskej@gmail.com)
 *
 * LEGAL: THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO
 * EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * DESCRIPTION: This Library communicates with a PC or other microcontroller.
 * 
 * All settings are defined in a config file (TODO) and loaded at build time. 
 * Any settings changed through communication are volatile.
 * -----------------------------------------------------------------------------
 * Communication Schema:
 * Send command then options, module replies with ack or MSB data
 * 
 * Commands:
 * 
 * move                 ['a']       // moves all channels concurrently TODO Bresenham
 * -0   steps           [int32]
 * -0   max steps/sec   [int32]
 * -1   steps           [int32]
 * -1   max steps/sec   [int32]
 * -2   steps           [int32]
 * -2   max steps/sec   [int32]
 * -3   steps           [int32]
 * -3   max steps/sec   [int32]
 * rtns:ack             [char]
 * 
 * config               ['e']       // Sets a single setting (volatile)
 * -channel             [uint8]     //index from 0
 * -setting             [char]      //accel, 
 * -option              []
 * rtns:ack             [char]
 * 
 * stop                 ['i']       // will slow down gracefully
 * -channel             [uint8]     //index from 0
 * rtns:ack             [char]
 * 
 * estop                ['j']       // cuts power to stepper driver
 * -channel             [uint8]     //index from 0
 * rtns:ack             [char]
 * 
 * -----------------------------------------------------------------------------
*/

#ifndef COMM_H
#define COMM_H

#include <main.h>

// send a 32 bit number MSB
// result[0] = (value & 0xff000000) >> 24;
// result[1] = (value & 0x00ff0000) >> 16;
// result[2] = (value & 0x0000ff00) >> 8;
// result[3] = (value & 0x000000ff);

/**
 * @brief commands to be sent by the master to the stepper controller
 * 
 */
#define MOVE 'a'
#define CONFIG 'e'
#define STOP 'i'
#define ESTOP 'j'

// Correspondes to the schema above
#define MAX_OPTION_LENGTH 3
#define MAX_RTN_LENGTH 2

/**
 * @brief A packet without any of the fun stuff figured out, this will be parsed be individual handlers
 * 
 */
struct Packet
{
    char command;
    uint16_t options[MAX_OPTION_LENGTH];
    uint16_t rtn[MAX_RTN_LENGTH];
    int numreturns;
} Packet;

/**
 * @brief will read a packet(hopefully) of bytes and
 * determine which handler needs to be called.
 * 
 * @param bytes probably a receive buffer
 * @param length 
 * @return int 0 if successful, -1 otherwise
 */
int parseSerialCommand(uint8_t bytes[], uint8_t length);

void handleMove(int32_t steps[], int32_t speeds[]);
void handleConfig(uint8_t channel);
void handleStop(uint8_t channel);
void handleEStop(uint8_t channel);

#endif
