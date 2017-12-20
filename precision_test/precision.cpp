/*******************************************************************************
 * slavenode.cpp
 *
 * This source file implements the "slave" node in the timestamp free protocol.
 *
 * M.Overdick, A.G.Klein, and J.Canfield
 * Last Major Revision: 8/5/2016
 ******************************************************************************/
#ifndef INCLUDES_HPP    // Prevents including file twice
#define INCLUDES_HPP

#define PI      3.14159265358979323846
#define TWOPI   6.28318530717958647692

/*******************************************************************************
* Generic Project Includes
******************************************************************************/
#include "../include/types.hpp"                        // Master type definition file
#include <iostream>                         // Input/Output to terminal
#include <fstream>                          // File input and output
#include <csignal>                          // Signal handlers
#include <cmath>                            // Mathematical funcitons
#include <math.h>                           // Math
#include <complex>                          // Complex numbers
#include <vector>                           // Standard vector type

/*******************************************************************************
* UHD Includes
******************************************************************************/
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>

/*******************************************************************************
* Boost Includes
******************************************************************************/
#include <boost/format.hpp>                 // String formatting lib
#include <boost/thread.hpp>                 // Thread handling lib
#include <boost/assign/list_of.hpp>

/*******************************************************************************
* Project Specific Includes
******************************************************************************/
#include "../include/sinc.hpp"                         // Sinc pulse generator
#include "../include/writebuff.hpp"                    // Writes buffer to file
#include "../include/checklo.hpp"                      // Checks LO Lock status
#endif

    // Compliation parameters
#define DEBUG           0                       // Debug (binary) if 1, debug code compiled


#define DURATION        180                     // Length of time to record in seconds

#define DELAY           60                      // Amount of time before recording starts in seconds

    // Radio Parameters
#define SAMPRATE        250e3                   // Sampling Rate (Hz)
#define CARRIERFREQ     0                       // Carrier Frequency (Hz)
#define CLOCKRATE       30.0e6                  // Clock Rate (Hz)
#define USRPIP          "addr=192.168.10.13"    // Ip string of USRP

    // Transmission parameters
#define SPB             1000                    // Samples Per Buffer SYNC_PERIOD
#define TXDELAY         15                       // Number of Buffers to Delay transmission (Must Be Odd)
#define CBW             0.5                     // Normalized Freq Offset of Sinc Pulse (1 --> Nyquist)
#define BW              CBW*0.9                 // Normalized Bandwidth of Sinc pulse (1 --> Nyquist)
#define SYNC_PERIOD     15                      // Sync Period (# of buffers, 11 is safe minimum)
#define SINC_PRECISION  10000                   // Precision of sinc pulse delays relative to SAMPRATE
                                                // Precision in seconds = 1/(SAMPRATE*SINC_PRECISION)

    // Sinc pulse amplitudes (integer)
#define TX_AMP          0x7FFF                  // Peak value of sinc pulse generated for debug channel (max 32768)
#define TOTAL_LENGTH    10000


std::vector< CINT16 >   rxbuff(SPB);
// FP64  test_vector[] = {-0.5,-0.49,-0.48,-0.47,-0.46,-0.45,-0.44,-0.43,-0.42,-0.41,-0.4,-0.39,-0.38,-0.37,-0.36,-0.35,-0.34,-0.33,-0.32,-0.31,-0.3,-0.29,-0.28,-0.27,-0.26,-0.25,-0.24,-0.23,-0.22,\
//   -0.21,-0.2,-0.19,-0.18,-0.17,-0.16,-0.15,-0.14,-0.13,-0.12,-0.11,-0.1,-0.09,-0.08,-0.07,-0.06,-0.05,-0.04,-0.03,-0.02,-0.01,0,0.01,0.02,0.03,0.04,0.05,0.06,0.07,0.08,0.09,0.1,0.11,0.12,0.13,0.14,0.15,0.16,0.17,\
//   0.18,0.19,0.2,0.21,0.22,0.23,0.24,0.25,0.26,0.27,0.28,0.29,0.3,0.31,0.32,0.33,0.34,0.35,0.36,0.37,0.38,0.39,0.4,0.41,0.42,0.43,0.44,0.45,0.46,0.47,0.48,0.49,0.5};
INT16U  test_length = 1;//101;

INT16U ping_ctr         = 0;                        // Counter for transmitting pulses
bool tx_ping            = true;

/*******************************************************************************
 * Signal handlers
 ******************************************************************************/
static bool stop_signal_called = false;
void sig_int_handler(int){stop_signal_called = true;}

/*******************************************************************************
 * Main function
 ******************************************************************************/
int UHD_SAFE_MAIN(int argc, char *argv[]){
    uhd::set_thread_priority_safe(1);

    /** Initialize USRP hardware **********************************************/
        // Create a USRP TX device
    uhd::usrp::multi_usrp::sptr usrp_tx = uhd::usrp::multi_usrp::make(std::string(USRPIP));
    usrp_tx->set_master_clock_rate(CLOCKRATE);                                              // set clock rate
    usrp_tx->set_clock_source(std::string("internal"));                                     // lock mboard clocks
    usrp_tx->set_time_source("none");                                                       // Use PPS signal
    usrp_tx->set_tx_subdev_spec(std::string("A:AB"));                                       // select the subdevice (2-channel mode)
    usrp_tx->set_tx_rate(SAMPRATE);                                                         // set the sample rate
    uhd::tune_request_t tune_request(CARRIERFREQ);                                          // validate tune request
    usrp_tx->set_tx_freq(tune_request,0);                                                   // set the center frequency of chan0
    usrp_tx->set_tx_antenna("TX/RX",0);                                                     // set the antenna of chan0
    boost::this_thread::sleep(boost::posix_time::seconds(1.0));                             // allow for some setup time

        // Create a USRP RX params
    uhd::usrp::multi_usrp::sptr usrp_rx = uhd::usrp::multi_usrp::make(std::string(USRPIP));
    usrp_rx->set_master_clock_rate(CLOCKRATE);                                              // set clock rate
    usrp_rx->set_clock_source(std::string("internal"));                                     // lock mboard clocks
    usrp_rx->set_time_source("none");                                                       // Use PPS signal
    usrp_rx->set_rx_subdev_spec(std::string("A:A"));                                        // select the subdevice
    usrp_rx->set_rx_rate(usrp_tx->get_tx_rate(),0);                                         // set the sample rate
    usrp_rx->set_rx_freq(tune_request,0);                                                   // set the center frequency
    usrp_rx->set_rx_antenna(std::string("RX2"),0);                                          // set the antenna
    boost::this_thread::sleep(boost::posix_time::seconds(1.0));                             // allow for some setup time

    /** Constant Decalartions *************************************************/
        // When recording, time specifies the number of buffers to record
    #if ((DEBUG != 0) && ((WRITERX != 0)||(WRITEXCORR != 0)||(WRITEKAL != 0))||(WRITETX != 0))
        INT16U write_ctr = 0;
        INT16U delay_ctr = 0;
        INT32U write_ctr_offset = 0;
        const INT32U time = DURATION*(usrp_tx->get_tx_rate()/SPB);
        const INT32U delay_time = DELAY*(usrp_tx->get_tx_rate()/SPB);
    #else
    #endif /* ((DEBUG != 0) && ((WRITERX != 0)||(WRITEXCORR != 0))) */

    /** Variable Declarations *************************************************/

        // create sinc pulses, zero, and receive buffers
    std::vector< CINT16 >   tx_sinc(SPB);               // Stores precomputed sinc pulse for transmission
    std::vector< CINT16 >   zero(SPB, (0,0));           // Stores all zeros for Tx

        // Initialize oversampled sinc pulse
    Sinc_Init(BW, CBW, SPB, SINC_PRECISION);

        // Generate synchronization and debug sinc pulses
    Sinc_Gen_TX(&tx_sinc.front(), TX_AMP, SPB, 0.0, tx_ping);


    /** Main code *************************************************************/
        // Check Ref and LO Lock detect for Tx
    std::vector<std::string> sensor_names;
    sensor_names = usrp_tx->get_tx_sensor_names(0);
    if (std::find(sensor_names.begin(), sensor_names.end(), "lo_locked") != sensor_names.end()) {
        uhd::sensor_value_t lo_locked = usrp_tx->get_tx_sensor("lo_locked",0);
        UHD_ASSERT_THROW(lo_locked.to_bool());
    }
    sensor_names = usrp_tx->get_mboard_sensor_names(0);

        // Check Ref and LO Lock detect for Rx
    check_locked_sensor(usrp_rx->get_rx_sensor_names(0), "lo_locked", boost::bind(&uhd::usrp::multi_usrp::get_rx_sensor, usrp_rx, _1, 0), 1.0);

        // Create a transmit streamer for TRX-A
    uhd::stream_args_t stream_args_tx0("sc16", "sc16");                         // Set transmit data type to Standard Complex 16-Bit (sc16)
    stream_args_tx0.channels = boost::assign::list_of(0);                       // Set number of channels
    uhd::tx_streamer::sptr tx_stream = usrp_tx->get_tx_stream(stream_args_tx0); // Create streamer

        // Create metadata handler for TRX-A
    uhd::tx_metadata_t md_tx;
    md_tx.start_of_burst    = true;
    md_tx.end_of_burst      = false;
    md_tx.has_time_spec     = true;

        // Create a receive streamer
    uhd::stream_args_t stream_args_rx("sc16", "sc16");                          // Set receive data type to Standard Complex 16-Bit (sc16)
    uhd::rx_streamer::sptr rx_stream = usrp_rx->get_rx_stream(stream_args_rx);  // Create Streamer
    uhd::rx_metadata_t md_rx;                                                   // Create metadata handler

        // report information to user (things which may differ from what was requested)
    std::cout << boost::format("Actual TX Rate: %f Msps...") % (usrp_tx->get_tx_rate()/1e6) << std::endl;
    std::cout << boost::format("Actual time between pulses: %f sec...") % (SYNC_PERIOD*SPB/usrp_tx->get_tx_rate()) << std::endl << std::endl;

        // set sigint so user can terminate via Ctrl-C
    std::signal(SIGINT, &sig_int_handler);
    std::cout << "Press Ctrl + C to stop streaming..." << std::endl;

        // setup receive streaming
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    stream_cmd.stream_now = false;
        // Tell USRP to start sampling 0.05 seconds in the future
    uhd::time_spec_t starttime = uhd::time_spec_t(0.05)+usrp_rx->get_time_now();
    stream_cmd.time_spec = starttime;
    rx_stream->issue_stream_cmd(stream_cmd);


    #if ((DEBUG != 0) && (WRITE_COUNTERS != 0))
        std::cout << "Delay time set to " << DELAY << " seconds" << std::endl;
        std::cout << "Write time set to " << DURATION << " seconds" << std::endl;
    #else
    #endif /* #if ((DEBUG != 0) && (WRITE_COUNTERS != 0)) */

    for(INT16U i = 0; i < test_length; i++){
        for(INT16 j = 0; j < TOTAL_LENGTH; j++){
            rx_stream->recv(&rxbuff.front(), SPB, md_rx, 3.0);

            md_tx.time_spec = md_rx.time_spec + uhd::time_spec_t((TXDELAY+1)*(SPB)/usrp_tx->get_tx_rate());

            if (ping_ctr >= SYNC_PERIOD-1) {
                tx_ping = true;                     // Set ping flag


                ping_ctr = 0;                       // Reset ping counter

            } else {
                tx_ping = false;                    // Clear ping flag
                ping_ctr++;                         // Increment counter

            }

                // Generate delayed pulse
            Sinc_Gen_TX(&tx_sinc.front(), TX_AMP, SPB, 0, 1);

                // Transmit both buffers
            tx_stream->send(&tx_sinc.front(), SPB, md_tx);
            md_tx.start_of_burst = false;



            /** Exit program to write buffers  ************************************/
            #if ((DEBUG != 0) && ((WRITERX != 0)||(WRITEXCORR != 0)||(WRITEKAL != 0)||(WRITESINC != 0)||(WRITETX != 0)))
                if(delay_ctr >= delay_time){
                    if(write_ctr >= time-1){
                        break;
                    }else{
                        write_ctr++;
                    }
                }else{
                    delay_ctr++;
                }
                    // Print Roundtrip time and Calculated Clockoffset to terminal
                #if (WRITE_COUNTERS != 0)
                    if(delay_ctr >= delay_time){
                        std::cout << boost::format("Write: %10.2f\r") % (write_ctr/(usrp_tx->get_tx_rate()/SPB)) << std::flush;
                    }else{
                        std::cout << boost::format("Delay: %10.2f\r") % (delay_ctr/(usrp_tx->get_tx_rate()/SPB)) << std::flush;
                    }
                #else
                #endif /* #if (WRITE_COUNTERS != 0) */
            #else
            #endif /* #if ((DEBUG != 0) && ((WRITERX != 0)||(WRITEXCORR != 0)||(WRITEKAL != 0)||(WRITESINC != 0)||(WRITETX != 0))) */
        }
    }

        // send a mini EOB packet
    md_tx.end_of_burst = true;
    tx_stream->send("", 0, md_tx);
    boost::this_thread::sleep(boost::posix_time::seconds(1.0));

    return EXIT_SUCCESS;
}   /** main() ****************************************************************/
