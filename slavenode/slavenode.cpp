/*******************************************************************************
 * slavenode.cpp
 *
 * This source file implements the "slave" node in the timestamp free protocol.
 *
 * M.Overdick, A.G.Klein, and J.Canfield
 * Last Major Revision: 8/5/2016
 ******************************************************************************/

#include "includes.hpp"
#include <numeric>

    // Compliation parameters
#define DEBUG           0                       // Debug (binary) if 1, debug code compiled

#define WRITESINC       1                       // Write Sinc (binary) if 1, debug sinc pulse
                                                // is written to file "./sinc.dat"

#define DURATION        30                     // Length of time to record in seconds

#define DELAY           5                      // Amount of time before recording starts in seconds

#define WRITEXCORR      0                       // Write cross correlation to file (binary)

#define WRITERX         1                       // Write receive buffer to file (binary)

#define WRITEKAL        1                       // Write Kalman filter components to

#define WRITETX         1                       // Write transmission buffer to file (binary)

#define WRITEREADME     1

    // Radio Parameters
#define SAMPRATE        250e3                   // Sampling Rate (Hz)
#define CARRIERFREQ     0                       // Carrier Frequency (Hz)
#define CLOCKRATE       30.0e6                  // Clock Rate (Hz)
#define USRPIP          "addr=192.168.10.13"    // Ip string of USRP
#define HW_CAL          -0.939e-3//-1.74e-3//0.443e-3//0.680e-3//-0.962e-3//.0012                  // Measured hardware delay calibration (tx_delays)

    // Kalman Filter Gains
#define KALGAIN1_START  0.6707//0.65                        // Gain for master clock time estimate (set to 1.0 to prevent Kalman update)
#define KALGAIN2_START  1.1698e-5//1e-6                    // Gain for master clock rate estimate (set to 0.0 to prevent Kalman update)
#define KALGAIN1_STOP   0.65                     // Gain for master clock time estimate (experimentally derived)
#define KALGAIN2_STOP   1e-6                    // Gain for master clock rate estimate (experimentally derived)
#define KALGAIN_UPDATE  0                       // Enables KALGAINX_START to converge to KALGAINX_STOP (0 for off, 1 for on)
#define RATE_DIFF       0              // The difference between the Master and Slave periods
#define RATE_SEED       (1.0 + RATE_DIFF)       // Seed value for rate_est (experimentally derived)
#define CLKRT           0.0                     // Clockrate estimate

    // Transmission parameters
#define SPB             1000                    // Samples Per Buffer SYNC_PERIOD
#define NRXBUFFS        3                       // Number of Receive Buffers (circular)
#define TXDELAY         3                       // Number of Buffers to Delay transmission (Must Be Odd)
#define CBW             0.8  //0.5                     // Normalized Freq Offset of Sinc Pulse (1 --> Nyquist)
#define BW              0.02   //CBW*0.9                 // Normalized Bandwidth of Sinc pulse (1 --> Nyquist)
#define SYNC_PERIOD     15                      // Sync Period (# of buffers, 11 is safe minimum)
#define SINC_PRECISION  10000                   // Precision of sinc pulse delays relative to SAMPRATE
                                               // Precision in seconds = 1/(SAMPRATE*SINC_PRECISION)

    // Sinc pulse amplitudes (integer)
#define XCORR_AMP       0x400                   // Peak value of sinc pulse generated for cross correlation
#define TX_AMP          0x7FFF                  // Peak value of sinc pulse generated for debug channel (max 32768)
#define THRESHOLD       15e5                     // Threshold of cross correlation pulse detection
#define XCORR_SHIFT     3                       // How many times to divide the cross correlation by 2 before normalizing

    // Calculation defines
#define FDEL_SCALE      1//(0.5/0.4477)
#define FDEL_THRESH     .48
#define STRINGIFY(x)    #x

    // Print values for debugging purposes
#define PING_TIMES      0                       // Prints ping times to the terminal
#define OFFSET_CALC     1                       // Prints the course and fine offsets to the terminal
#define WRITE_COUNTERS  1                       // Prints the write and delay times to the terminal (Requires debug to be enabled and at least one WRITE debug)

#define NUM_DEFINES     25                      // The number of defines written to file, must be updated when more defines are added.
    // Structure for handling pulse detections
typedef struct {
            INT32U center_pos;                  // Position of pulse within buffer
            CINT64 center;                      // Maximum value detected pulse (cross correlation output)
        } MAXES;

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
    usrp_rx->set_rx_subdev_spec(std::string("A:B"));                                        // select the subdevice
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

    /** Debugging Variables Initialization ************************************/
    #if ((DEBUG != 0) && (WRITETX != 0))
        std::vector<CINT16> tx_sinc_rec(SPB*time);
        std::vector< FP64 > tx_time_rec(time);
    #else
    #endif /* #if ((DEBUG != 0) && (WRITETX != 0)) */

    #if ((DEBUG != 0) && (WRITERX != 0))
        std::vector<CINT16> rx_sinc_rec(SPB*time);
        std::vector< FP64 > rx_time_rec(time);
    #else
    #endif /* #if ((DEBUG != 0) && (WRITERX != 0)) */

    #if ((DEBUG != 0) && (WRITEKAL != 0))
            // Kalman filter variablespred_error
        std::vector< FP64 > clockoffset_rec(time);      // stores clockoffset
        std::vector< FP64 > pred_master_rec(time);      // stores pred_master
        std::vector< FP64 > pred_error_rec(time);       // stores pred_error

        std::vector< FP64 > time_est_rec(time);         // stores time_est
        std::vector< FP64 > rate_est_rec(time);         // stores rate_est
        std::vector< FP64 > time_pred_rec(time);        // stores time_pred
        std::vector< FP64 > rate_pred_rec(time);        // stores rate_pred

        std::vector< FP64 > fdel_rec(time);             // stores fdel

        std::vector<CINT64> exact_max_center_rec(time); // stores center_max
        std::vector< FP64 > roundtrip_rec(time);        // store roundtrip time
        std::vector< FP64 > buff_timer_rec(time);   // stores the flag set when previous is used
    #else
    #endif /* #if ((DEBUG != 0) && (WRITEKAL != 0)) */

    #if ((DEBUG != 0) && (WRITEREADME != 0))
        std::vector< FP64 > define_dat_rec(NUM_DEFINES);
        std::ofstream read_me_file;
    #else
    #endif /* #if ((DEBUG != 0) && (WRITEREADME != 0)) */

        // Only compiled when debugging
    #if ((DEBUG != 0) && (WRITEXCORR != 0))
        std::vector<CINT64> xcorr_write(SPB*time);      // Xcorr variable
    #else
    #endif /* #if ((DEBUG != 0) && (WRITEXCORR != 0)) */

    /** Variable Declarations *************************************************/

        // create sinc pulses, zero, and receive buffers
    std::vector< CINT64 >   xcorr_sinc(SPB);            // Stores precomputed sinc pulse for cross correlation  (constant)
    std::vector< CINT16 >   tx_sinc(SPB);               // Stores precomputed sinc pulse for transmission
    std::vector< CINT16 >   zero(SPB, (0,0));           // Stores all zeros for Tx
    std::vector< CINT16 >   rxbuff(NRXBUFFS*SPB);     // Circular receive buffer, keeps most recent 3 buffers
    std::vector< CINT16 *>  rxbuffs(NRXBUFFS);        // Vector of pointers to sectons of rx_buff

        // Correlation variables
    CINT64 xcorr;                                       // Cross correlation
    bool threshbroken = false;                          // Threhold detection
    bool calculate    = false;                          // Calculate delay trigger

        // Delay estimation variables
    MAXES crnt_max, prev_max, exact_max;                // Xcorr max structures
    FP64 fdel               = 0.0;                      // Position of peak (in fractional samples)
    FP64 clockoffset        = 0.0;                      // Calculated offset of master to slave (in samples)
    FP64 clkrt_ctr          = 0.0;                      // Calculated clockrate (manually set)

        // Kalman filter variables
    FP64  pred_master        = 0.0;                     // Current time of master
    FP64  time_est           = 0.0;                     // Time estimate of master
    FP64  time_pred          = 0.0;                     // Time prediction of master
    FP64  rate_est           = 1.0;                     // Rate estimate of master
    FP64  rate_pred          = 1.0;                     // Rate prediction of master
    FP64  pred_error         = 0.0;                     // Prediction error
    FP64  pred_error_unfixed = 0.0;
    FP64  elapsed_time       = 0;
    FP64  roundtrip          = 0;
    INT32 buff_timer         = 0;                       // Timer for time between transmitting and receiving a pulse
    INT32 buff_timer_grab    = 0;
    INT32 bt_avg             = 0;                       // Moving average variable
    INT32 current_slave_time = 0;                       // The current time of the slavenode when the calculation starts relative to a transmition
    std::vector< INT32 >  bt_vec(3);                    // Moving average for buff timer

    FP64  k_gain1           = KALGAIN1_START;           // Kalman gain 1
    FP64  k_gain2           = KALGAIN2_START;           // Kalman gain 2

        // Counters
    INT16U ping_ctr         = 0;                        // Counter for transmitting pulses
    INT16U rxbuff_ctr       = 0;                        // Counter for circular rx buffer
    INT16U i,j,k;                                       // Generic counters

        // Book keeping
    bool first_calc         = true;                     // First time running calculate code
    bool tx_ping            = true;                     // Transmit ping pulse
    bool fdel_is_stable     = true;

    /** Variable Initializations **********************************************/
        // Initialise rxbuffs (Vector of pointers)
    for(i = 0; i < NRXBUFFS; i++){
        rxbuffs[i] = &rxbuff.front() + SPB * i;
    }

        // Initialize oversampled sinc pulse
    Sinc_Init(BW, CBW, SPB, SINC_PRECISION);

        // Generate sinc pulse for cross correlation
    Sinc_Gen_XC(&xcorr_sinc.front(), XCORR_AMP, SPB);

        // Generate synchronization and debug sinc pulses
    Sinc_Gen_TX(&tx_sinc.front(), TX_AMP, SPB, 0.0, tx_ping);

    /** Debug code for writing sinc pulse *************************************/
        // Write template sinc pulse to file for debug
    #if ((DEBUG != 0) && (WRITESINC != 0))
        std::cout << "Writing xcorr Sinc to file..." << std::flush;
        writebuff("./xcorr_sinc.dat", &xcorr_sinc.front(), SPB);
        std::cout << "done!" << std::endl;
    #else
    #endif /* #if ((DEBUG != 0) && (WRITESINC != 0)) */

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

        // grab initial block of received samples from USRP with nice long timeout (gets discarded)
    rx_stream->recv(rxbuffs[0], SPB, md_rx, 3.0);

    #if ((DEBUG != 0) && (WRITE_COUNTERS != 0))
        std::cout << "Delay time set to " << DELAY << " seconds" << std::endl;
        std::cout << "Write time set to " << DURATION << " seconds" << std::endl;
    #else
    #endif /* #if ((DEBUG != 0) && (WRITE_COUNTERS != 0)) */

    while(not stop_signal_called){
        /***********************************************************************
         * SEARCHING block - (cross correlating and pulse detection)
         **********************************************************************/
            // Increment number of frames elapsed since last pulse was received from master
        buff_timer++;

            // Remember current max values
        prev_max = crnt_max;

            // Grab block of received samples from USRP
        rx_stream->recv(rxbuffs[rxbuff_ctr], SPB, md_rx);

            // Dumps the rxbuffs to a recording buffer for later
        #if ((DEBUG != 0) && (WRITERX != 0))
            write_ctr_offset = (write_ctr*SPB);

              // Save tx
            rx_time_rec[write_ctr] = (usrp_rx->get_time_now()).get_real_secs();
            for(INT16U n = 0; n < SPB; n++){
                  rx_sinc_rec[(write_ctr_offset)+n] = rxbuffs[rxbuff_ctr][n];
            }
        #else
        #endif /* #if ((DEBUG != 0) && (WRITERX != 0)) */

        /** CROSS CORRELATION, FIND PEAK **************************************/
            // Clear data structure elements
        crnt_max.center = CINT64(0,0);
        crnt_max.center_pos = 0;

            // Compute cross-correlation
        for (i = 0; i < SPB; i++) {
            xcorr = 0;  // Initialize xcorr variable

                // Cross correlation for circular buffer
            if(rxbuff_ctr == 0){
                for(j = 0; j < SPB-1-i; j++){
                    xcorr += (CINT64)rxbuffs[NRXBUFFS-1][i+j+1].real() * (CINT64)xcorr_sinc[j];
                }
                for(j = SPB-1-i; j < SPB; j++){
                    xcorr += (CINT64)rxbuffs[0][-SPB+1+i+j].real() * (CINT64)xcorr_sinc[j];
                }
            }else{
                for (j = 0; j < SPB; j++) {
                    xcorr += (CINT64)rxbuffs[rxbuff_ctr-1][i+j+1].real() * (CINT64)xcorr_sinc[j];
                }
            }

                // Find max value of cross correlation and save its characteristics
            if(std::abs(xcorr.real()) > std::abs(crnt_max.center.real())){
                crnt_max.center = xcorr;                    // Save crnt_max value
                crnt_max.center_pos = i;                    // Save crnt_max position
            }else{}

            /** Save buffers if enabled by defines ****************************/
                // Save normxcorr if enabled by defined variables
            #if ((DEBUG != 0) && (WRITEXCORR != 0))
                xcorr_write[(SPB*write_ctr)+i] = xcorr;
            #else
            #endif /* #if ((DEBUG != 0) && (WRITEXCORR != 0)) */
        }
            // Trigger calculation block after extra buffer
        if (threshbroken == true){
            calculate = true;           // Enable calculation of peak position
            threshbroken = false;       // Clear flag
            // Receive and correlate one extra buffer to ensure peak is caught
        }else if (std::abs(crnt_max.center.real()) >= THRESHOLD){
            threshbroken = true;        // Set flag
        }else{}

            // Increment rx buffer counter
        rxbuff_ctr++;
        if (rxbuff_ctr >= NRXBUFFS) {
            rxbuff_ctr = 0;
        }else{}

        /***********************************************************************
         * CALCULATING block - Finds delay and synchronizes
         **********************************************************************/
        if(calculate == true){
                // Print ping times to terminal
            #if (PING_TIMES != 0)
                std::cout << boost::format("Ping RX Time %10.5f") % (md_rx.time_spec.get_full_secs() + md_rx.time_spec.get_frac_secs()) << std::endl;
            #else
            #endif /* #if ((DEBUG != 0) && (PING_TIMES != 0)) */

                // Update the current slave time
            current_slave_time = (buff_timer * SPB);
            buff_timer_grab = buff_timer;

                // Did previous buffer have largest peak?
            if(std::abs(prev_max.center.real()) > std::abs(crnt_max.center.real())){        // YES
                exact_max = prev_max;
                buff_timer--;           // Correct timer
            }else{                                                                          // NO
                exact_max = crnt_max;
            }

            /** Delay estimator (atan2 fractional delay & Kalman filter) ******/

                // Negate center value if real value is negative
            if(exact_max.center.real() < 0){
                std::cout << "Center value was negative, had to flip." << std::endl;
                std::cout << k << std::endl;
                k++;
                exact_max.center = CINT64(-1,0)*exact_max.center;
            }else{}

                // Calculate fractional offset
            fdel = FDEL_SCALE*(std::atan2((FP64)exact_max.center.imag(), (FP64)exact_max.center.real())/(FP64)(CBW*PI));

                // Calculated using the roundtrip time

            clockoffset = (((FP64)((buff_timer - 1) * SPB) + (FP64)((FP64)exact_max.center_pos - SPB) + fdel)/2) + 1;

            elapsed_time = (FP64)current_slave_time - clockoffset;
            //std::cout << current_slave_time << std::endl;
            //std::cout << (FP64)(((buff_timer - 1) * SPB) + (FP64)((FP64)exact_max.center_pos - SPB) + fdel) << std::endl;

                // Print Roundtrip time and Calculated Clockoffset to terminal
            #if (OFFSET_CALC != 0)
                std::cout << boost::format("Roundtrip Time: %10.2f") % (((FP64)((buff_timer - 1) * SPB) + (FP64)((FP64)exact_max.center_pos - SPB) + fdel)) << std::flush;
                std::cout << boost::format("\tClockoffset: %10.2f") % (clockoffset) << std::endl;
            #else
            #endif /* #if (OFFSET_CALC != 0) */

                // Moving average of buff timer (Prevents glitching if pulse is lost)
                // Useful for the start of the code while device calibrates
            bt_vec.insert(bt_vec.end(),buff_timer);
            bt_vec.erase(bt_vec.begin());
            bt_avg = std::accumulate(bt_vec.begin(),bt_vec.end(),0)/bt_vec.size();

                //Errors trickle through imperfect xcorr calculation, arctan is
            if(std::abs(fdel) > FDEL_THRESH){
              fdel_is_stable = false;
            }else{
              fdel_is_stable = true;
            }

                // Only update Kalman filter when buff_timer is consistent
            if((bt_avg == buff_timer) && (fdel_is_stable)){
                    // Kalman Filter

                pred_master = -elapsed_time - (FP64)((TXDELAY + 1)*SPB);    // Calculate current time at master


                    // Wrap the pred_master to be within +/- SPB/2
                while(pred_master<=-(SPB/2)){
                    pred_master = pred_master+(FP64)(SPB*rate_est);
                }
                while(pred_master>(SPB/2)){
                    pred_master = pred_master-(FP64)(SPB*rate_est);
                }

                pred_error  = pred_master - time_pred;              // Calculate error of prediction

                    // Make sure pred_error is within +/- SPB/2
                while(pred_error >= (SPB/2)){
                    std::cout << pred_error << " pred_error wrapped " << std::endl;
                    pred_error -= SPB*rate_est;
                }
                while(pred_error < -(SPB/2)){
                    std::cout << pred_error << " pred_error wrapped " << std::endl;
                    pred_error += SPB*rate_est;
                }

                    // Update Estimates
                time_est  = time_pred + k_gain1 * pred_error;       // Estimate time of master
                rate_est  = rate_pred + k_gain2 * pred_error;       // Estimate clock rate of master

                    // Slowly decrease kalman gain to converge to KALGAINX_STOP
                // if(k_gain1 > KALGAIN1_STOP){
                //    k_gain1 = k_gain1*0.99;
                // }else{}
                // if(k_gain2 > KALGAIN2_STOP){
                //    k_gain2 = k_gain2*0.9;
                // }else{}

                    // On first calc run only, to keep rate_est from straying too far from 1
                if(first_calc){
                    rate_est = RATE_SEED;
                    first_calc = false;
                }else{}

                    // Update Predictions
                rate_pred = rate_est;                               // Predict rate of master

            }else{
                    // If buff_timer is unstable, use predicted values for update
                time_est  = time_pred;                              // Update estimate with prediction
                rate_est  = rate_pred;                              // Update rate estimate with prediction
            }

                // Exit calculating mode
            calculate = false;

            // When there is no pulse received, the update KF using predicted values
        }else{
            time_est  = time_pred;                                  // Update estimate with prediction
            rate_est  = rate_pred;                                  // Update rate estimate with prediction
        } /********** if(calculate == true) *****************/

            // wrap time_est if need be
        while(time_est >= (SPB/2)){
            std::cout << time_est << " time_est wrapped " << std::endl;
            time_est -= SPB*rate_est;
        }
        while(time_est < -(SPB/2)){
            std::cout << time_est << " time_est wrapped " << std::endl;
            time_est += SPB*rate_est;
        }

        time_pred = time_est + (rate_est - 1) * (FP64)SPB;        // Predict time of master


        /***********************************************************************
         * TRANSMITTING block - Transmits debug signal and "ping"
         **********************************************************************/

            // Compute tx time_spec
        md_tx.time_spec = md_rx.time_spec + uhd::time_spec_t((TXDELAY+1)*(SPB)/usrp_tx->get_tx_rate());

            // Transmit ping every SYNC_PERIOD buffers
        if (ping_ctr >= SYNC_PERIOD-1) {
            tx_ping = true;                     // Set ping flag

            buff_timer = -TXDELAY;              // Initiate buff_timer

            ping_ctr = 0;                       // Reset ping counter

        } else {
            tx_ping = false;                    // Clear ping flag
            ping_ctr++;                         // Increment counter

        }

            // Generate delayed pulse
        Sinc_Gen_TX(&tx_sinc.front(), TX_AMP, SPB, time_est*rate_est + (SPB/2)*rate_est - (SPB/2) + HW_CAL, tx_ping);

            // Transmit both buffers
        tx_stream->send(&tx_sinc.front(), SPB, md_tx);
        md_tx.start_of_burst = false;

            // Move trasmitted buffers into a recording buffer
        #if ((DEBUG != 0) && (WRITETX != 0))
            write_ctr_offset = (write_ctr*SPB);

            tx_time_rec[write_ctr] = (usrp_rx->get_time_now()).get_real_secs();
            for(INT16U i = 0; i < SPB; i++){
                tx_sinc_rec[(write_ctr_offset)+i] = tx_sinc[i];
            }
        #else
        #endif /* #if ((DEBUG != 0) && (WRITETX != 0))


        /** Copy Kalman variables *********************************************/
        #if ((DEBUG != 0) && (WRITEKAL != 0))
            clockoffset_rec[write_ctr]  =   clockoffset;    // stores clockoffset
            pred_master_rec[write_ctr]  =   pred_master;    // stores pred_master
            pred_error_rec[write_ctr]   =   pred_error;     // stores pred_error

            time_est_rec[write_ctr]     =   time_est;       // stores time_est
            rate_est_rec[write_ctr]     =   rate_est;       // stores rate_est
            time_pred_rec[write_ctr]    =   time_pred;      // stores time_pred
            rate_pred_rec[write_ctr]    =   rate_pred;      // stores rate_pred

            fdel_rec[write_ctr]         =   fdel;           // stores fdel
            roundtrip_rec[write_ctr]    =   roundtrip; // store roundtrip time

            exact_max_center_rec[write_ctr]   =   exact_max.center;
            buff_timer_rec[write_ctr]   =   buff_timer_grab;
        #else
        #endif /* #if ((DEBUG != 0) && (WRITEKAL != 0)) */

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
    }   /** while(not stop_signal_called) *************************************/

        // Write template sinc pulse to file for debug

        /** Write buffers *****************************************************/

        // Write tx buffer to file
    #if ((DEBUG != 0) && (WRITETX != 0))
        std::cout << "Writing tx buffer to file..." << std::flush;
        writebuff("./tx.dat", &tx_sinc_rec[0], SPB*write_ctr);
        std::cout << "done!" << std::endl;

        std::cout << "Writing tx_time to file..." << std::flush;
        writebuff("./tx_time.dat", &tx_time_rec[0], write_ctr);
        std::cout << "done!" << std::endl;
    #else
    #endif /* #if ((DEBUG != 0) && (WRITETX != 0)) */

      // Write rx buffer to file
    #if ((DEBUG != 0) && (WRITERX != 0))
        std::cout << "Writing rx buffer to file..." << std::flush;
        writebuff("./rx.dat", &rx_sinc_rec[0], SPB*write_ctr);
        std::cout << "done!" << std::endl;

        std::cout << "Writing rx_time buffer to file..." << std::flush;
        writebuff("./rx_time.dat", &rx_time_rec[0], write_ctr);
        std::cout << "done!" << std::endl;
    #else
    #endif /* #if ((DEBUG != 0) && (WRITERX != 0)) */


    #if ((DEBUG != 0) && (WRITEXCORR != 0))
        std::cout << "Writing cross correlation to file..." << std::flush;
        writebuff("./xcorr.dat", &xcorr_write.front(), SPB*write_ctr);
        std::cout << "done!" << std::endl;
    #else
    #endif /* #if ((DEBUG != 0) && (WRITEXCORR != 0)) */

    #if ((DEBUG != 0) && (WRITEKAL != 0))
        std::cout << "Writing clockoffset to file..." << std::flush;
        writebuff("./clockoffset.dat", &clockoffset_rec.front(), write_ctr);
        std::cout << "done!" << std::endl;

        std::cout << "Writing pred_master to file..." << std::flush;
        writebuff("./pred_master.dat", &pred_master_rec.front(), write_ctr);
        std::cout << "done!" << std::endl;

        std::cout << "Writing pred_error to file..." << std::flush;
        writebuff("./pred_error.dat", &pred_error_rec.front(), write_ctr);
        std::cout << "done!" << std::endl << std::endl;



        std::cout << "Writing time_est to file..." << std::flush;
        writebuff("./time_est.dat", &time_est_rec.front(), write_ctr);
        std::cout << "done!" << std::endl;

        std::cout << "Writing rate_est to file..." << std::flush;
        writebuff("./rate_est.dat", &rate_est_rec.front(), write_ctr);
        std::cout << "done!" << std::endl;

        std::cout << "Writing time_pred to file..." << std::flush;
        writebuff("./time_pred.dat", &time_pred_rec.front(), write_ctr);
        std::cout << "done!" << std::endl;

        std::cout << "Writing rate_pred to file..." << std::flush;
        writebuff("./rate_pred.dat", &rate_pred_rec.front(), write_ctr);
        std::cout << "done!" << std::endl << std::endl;

        std::cout << "Writing roundtrip to file..." << std::flush;
        writebuff("./roundtrip.dat", &roundtrip_rec.front(), write_ctr);
        std::cout << "done!" << std::endl << std::endl;

        std::cout << "Writing buff_timer to file..." << std::flush;
        writebuff("./buff_timer.dat", &buff_timer_rec.front(), write_ctr);
        std::cout << "done!" << std::endl << std::endl;


        std::cout << "Writing fdel to file..." << std::flush;
        writebuff("./fdel.dat", &fdel_rec.front(), write_ctr);
        std::cout << "done!" << std::endl;

        std::cout << "Writing exact_max_center to file..." << std::flush;
        writebuff("./exact_max_center.dat", &exact_max_center_rec.front(), write_ctr);
        std::cout << "done!" << std::endl;
    #else
    #endif /* #if ((DEBUG != 0) && (WRITEKAL != 0)) */

    /** Write Define Values to File ***************************************/
    #if ((DEBUG != 0) && (WRITEREADME != 0))
        read_me_file.open("README.txt");

        read_me_file << STRINGIFY(SAMPRATE)         << "\t" << SAMPRATE << std::endl;
        define_dat_rec[0] =                         (FP64) SAMPRATE;

        read_me_file << STRINGIFY(CARRIERFREQ)      << "\t" << CARRIERFREQ << std::endl;
        define_dat_rec[1] =                         (FP64) CARRIERFREQ;

        read_me_file << STRINGIFY(CLOCKRATE)        << "\t" << CLOCKRATE << std::endl;
        define_dat_rec[2] =                         (FP64) CLOCKRATE;

        read_me_file << STRINGIFY(HW_CAL)           << "\t" << HW_CAL << std::endl;
        define_dat_rec[3] =                         (FP64) HW_CAL;


        read_me_file << STRINGIFY(KALGAIN1_START)   << "\t" << KALGAIN1_START << std::endl;
        define_dat_rec[4] =                         (FP64) KALGAIN1_START;

        read_me_file << STRINGIFY(KALGAIN2_START)   << "\t" << KALGAIN2_START << std::endl;
        define_dat_rec[5] =                         (FP64) KALGAIN2_START;

        read_me_file << STRINGIFY(KALGAIN1_STOP)    << "\t" << KALGAIN1_STOP << std::endl;
        define_dat_rec[6] =                         (FP64) KALGAIN1_STOP;

        read_me_file << STRINGIFY(KALGAIN2_STOP)    << "\t" << KALGAIN2_STOP << std::endl;
        define_dat_rec[7] =                         (FP64) KALGAIN2_STOP;

        read_me_file << STRINGIFY(KALGAIN_UPDATE)   << "\t" << KALGAIN_UPDATE << std::endl;
        define_dat_rec[8] =                         (FP64) KALGAIN_UPDATE;

        read_me_file << STRINGIFY(RATE_DIFF)        << "\t" << RATE_DIFF << std::endl;
        define_dat_rec[9] =                         (FP64) RATE_DIFF;

        read_me_file << STRINGIFY(RATE_SEED)        << "\t" << RATE_SEED << std::endl;
        define_dat_rec[10] =                        (FP64) RATE_SEED;

        read_me_file << STRINGIFY(CLKRT)            << "\t" << CLKRT << std::endl;
        define_dat_rec[11] =                        (FP64) CLKRT;


        read_me_file << STRINGIFY(SPB)              << "\t" << SPB << std::endl;
        define_dat_rec[12] =                        (FP64) SPB;

        read_me_file << STRINGIFY(NRXBUFFS)         << "\t" << NRXBUFFS << std::endl;
        define_dat_rec[13] =                        (FP64) NRXBUFFS;

        read_me_file << STRINGIFY(TXDELAY)          << "\t" << TXDELAY << std::endl;
        define_dat_rec[14] =                        (FP64) TXDELAY;

        read_me_file << STRINGIFY(CBW)              << "\t" << CBW << std::endl;
        define_dat_rec[15] =                        (FP64) CBW;

        read_me_file << STRINGIFY(BW)               << "\t" << BW << std::endl;
        define_dat_rec[16] =                        (FP64) BW;

        read_me_file << STRINGIFY(SYNC_PERIOD)      << "\t" << SYNC_PERIOD << std::endl;
        define_dat_rec[17] =                        (FP64) SYNC_PERIOD;

        read_me_file << STRINGIFY(SINC_PRECISION)   << "\t" << SINC_PRECISION << std::endl;
        define_dat_rec[18] =                        (FP64) SINC_PRECISION;


        read_me_file << STRINGIFY(XCORR_AMP)        << "\t" << XCORR_AMP << std::endl;
        define_dat_rec[19] =                        (FP64) XCORR_AMP;

        read_me_file << STRINGIFY(TX_AMP)           << "\t" << TX_AMP << std::endl;
        define_dat_rec[20] =                        (FP64) TX_AMP;

        read_me_file << STRINGIFY(THRESHOLD)        << "\t" << THRESHOLD << std::endl;
        define_dat_rec[21] =                        (FP64) THRESHOLD;

        read_me_file << STRINGIFY(XCORR_SHIFT)      << "\t" << XCORR_SHIFT << std::endl;
        define_dat_rec[22] =                        (FP64) XCORR_SHIFT;


        read_me_file << STRINGIFY(FDEL_SCALE)       << "\t" << FDEL_SCALE << std::endl;
        define_dat_rec[23] =                        (FP64) FDEL_SCALE;

        read_me_file << STRINGIFY(FDEL_THRESH)      << "\t" << FDEL_THRESH << std::endl;
        define_dat_rec[24] =                        (FP64) FDEL_THRESH;

        read_me_file.close();

        std::cout << "Writing define_dat_rec to file..." << std::flush;
        writebuff("./define_dat_rec.dat", &define_dat_rec.front(), NUM_DEFINES);
        std::cout << "done!" << std::endl;

    #else
    #endif /* #if ((DEBUG != 0) && (WRITEREADME != 0)) */

        // send a mini EOB packet
    md_tx.end_of_burst = true;
    tx_stream->send("", 0, md_tx);
    boost::this_thread::sleep(boost::posix_time::seconds(1.0));

    return EXIT_SUCCESS;
}   /** main() ****************************************************************/
