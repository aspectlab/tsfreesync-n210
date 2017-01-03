/*******************************************************************************
 * sinc.cpp - Modulated sinc pulse generator code. Specific to N210 hardware.
 * This version of sinc.cpp generates the same sinc pulse for both the I and Q
 * channels.
 *
 * M.Overdick, J.Canfield, and A.G. Klein
 * Last Major Revision: 10/17/2016
 ******************************************************************************/
#include "includes.hpp"
#include <iostream>
#include <fstream>

#define SCALAR          0x7FFF       // Scalar for precomputed sinc pulse amplitude

    // Sinc macro
#define SINC(x)         ( std::sin(x) / (x) )

    // Initializes oversampled sinc pulse
extern void Sinc_Init(FP32 bw, FP32 cbw, INT32U spb, INT16U ratio);

    // Creates template sinc pulse for cross correlation
void Sinc_Gen_XC(CINT16 * table, INT16 ampl, INT16U spb);

    // Undersamples and creates centered pulse on I channel and a delayed pulse
    // on Q channel.
extern void Sinc_Gen_TX(CINT16 * table, INT16U ampl, INT16U spb, FP32 tau, bool I_en);

    // Checks for existance of "OS_Sinc.dat" and copy contents
static bool Sinc_Read();

static std::vector< CINT16 > Sinc_Table;    // Vector that holds oversampled pulse
static INT16U Sinc_Ratio;                   // Ratio between real sampling frequency
                                            // and oversampling frequency

/*******************************************************************************
 * Sinc_Init() - Innitialized sinc pulse generation, must be called before
 * caling Sinc_Gen(). This function creates an oversampled sinc pulse used to
 * generate undersampled sinc pulses with a programmable precision.
 *
 * ARGUMENTS:
 *      FP32 bw         -   Bandwidth of the sinc pulse, relative to fs
 *      FP32 cbw        -   Carrier bandwidth, relative to fs
 *      INT32U spb      -   Samples per buffer
 *      INT16U ratio    -   Oversampling ratio
 *
 *
 * RETURNS:(none)
 *
 *
 * M.Overdick, J.Canfield, and A.G. Klein
 * Last Major Revision: 7/21/2016
 ******************************************************************************/
void Sinc_Init(FP32 bw, FP32 cbw, INT32U spb, INT16U ratio){
     INT32U length  =   0;          // The length (n samples) of sinc pulse
     FP32   w0      =   0;          // Normalized bandwidth of carrier
     FP32   eta     =   0;          // Normalized bandwidth of sinc pulse
     INT64  j, i;                   // Counter

     Sinc_Ratio = ratio;            // Save ratio for external access

     length = spb*Sinc_Ratio;       // Compute length
     Sinc_Table.resize(length);     // Resize Sinc_Table vector to "length"

        // Check if sinc pulse has already been computed before.
     if(!Sinc_Read()){
        std::cout << "Generating oversampled sinc pulse..." << std::endl;

            // Compute normalized bandwidths
        w0 = cbw*PI;
        eta = bw*PI;

        for(j = 0; j < length; j++){
            i = j/Sinc_Ratio - spb/2;    // i is the oversampled step size

                // When i is not zero, generate sinc pulse as usual
            if(i != 0){
                Sinc_Table[j] = CINT16( SCALAR * ( std::sin(eta*i) / (eta*i) ) * std::cos(w0*i), \
                                        SCALAR * ( std::sin(eta*i) / (eta*i) ) * std::sin(w0*i));

                // Avoid divide by zero when i is zero
            }else{
                Sinc_Table[j] = CINT16(SCALAR, 0);
            }

                // Display percentage to prove the program hasn't crahsed
            std::cout << boost::format("\r\t%3.1f Percent Complete") % (FP32(100*j)/FP32(length)) << std::flush;
        }

        std::cout << "\r\tdone!                    " << std::endl << std::endl;

            // Write template sinc pulse to file
        std::cout << "Writing oversampled Sinc to file..." << std::flush;
        writebuff < CINT16 >("./OS_Sinc.dat", &Sinc_Table.front(), Sinc_Table.size());
        std::cout << "done!" << std::endl << std::endl;
    }else{
            // If OS_Sinc.dat is present, tell the user...
        std::cout << "Using provided oversampled sinc pulse from file." << std::endl;
        std::cout << "\tDelete \"OS_Sinc.dat\" to regenerate file." << std::endl << std::endl;
    }
 }

/*******************************************************************************
 * Sinc_Gen_TX() - Loads array with delayed sinc pulse.
 *
 * ARGUMENTS:
 *     CINT16* table   -   A pointer to the first element of an array of length
 *                         spb.
 *     INT16   ampl    -   The peak amplitude of the delayed sinc pulse, choose
 *                         a number between 0 - 32767.
 *     INT16U  spb     -   Samples per buffer.
 *     FP32    tau     -   The delay of the pulse for Q channel,
 *                         between +/- spb/2, 0. I channel has constant delay.
 *                         results in a centered pulse.
 *     bool  I_en      -   1 writes pulse, 0 writes zero to channel 1 (I channel)
 *
 * RETURNS:
 *     A centered sinc pulse in I channel and a delayed sinc pulse in Q channel
 *
 * M.Overdick, J.Canfield, and A.G. Klein
 * Last Major Revision: 10/17/2016
 ******************************************************************************/
void Sinc_Gen_TX(CINT16 * table, INT16U ampl, INT16U spb, FP32 tau, bool I_en){
    INT32   i       = 0;                    // Counter
    INT32   j       = 0;                    // Counter
    INT32   k       = 0;                    // Counter
    INT32U  shift   = 0;                    // Starting point of undersampling
    FP32    scale   = 0;                    // Scale variable

    scale = FP32(ampl)/SCALAR;    // Scalar for pulse amplitude

        // Invert tau direction
    tau = -tau;

        // Make shift always positive
    while(tau >= spb){
        tau -= spb;
    }
    while(tau < 0){
        tau += spb;
    }

    shift = tau*Sinc_Ratio;        // Compute starting point of undersampling

        /** Generate Pulse ****************************************************/

    j = shift;
    for(i = 0; i < spb*Sinc_Ratio; i = i + Sinc_Ratio){
        table[k].real() = Sinc_Table[i].real()*scale*I_en;
        table[k].imag() = Sinc_Table[j].real()*scale;

        k++;
        j = j + Sinc_Ratio;
            // Wrap around indexing of delayed pulse
        if(j >= spb*Sinc_Ratio){
            j = j - spb*Sinc_Ratio;
        }else{}
    }
}

/*******************************************************************************
 * Sinc_Gen_XC() - Loads array with complex sinc pulse
 *
 * ARGUMENTS:
 *     CINT16* table   -   A pointer to the first element of an array of length
 *                         spb.
 *     INT16   ampl    -   The peak amplitude of the delayed sinc pulse, choose
 *                         a number between 0 - 32767.
 *     INT16U  spb     -   Samples per buffer.
 *
 * RETURNS:
 *     A template sinc pulse for cross correlation
 *
 * M.Overdick, J.Canfield, and A.G. Klein
 * Last Major Revision: 10/17/2016
 ******************************************************************************/
void Sinc_Gen_XC(CINT16 * table, INT16 ampl, INT16U spb){
        // Counters
    INT32   i       = 0;                    // Counter
    INT32U  j       = 0;                    // Counter
    INT32U  remain  = 0;                    // Leftover space in over sample vector
    FP32    scale   = FP32(ampl)/SCALAR;    // Scalar for pulse amplitude

        // Generate first part of delayed pulse
    for(i = 0; i < spb*Sinc_Ratio; i = i + Sinc_Ratio){
        table[j] = CINT16(Sinc_Table[i].real()*scale, Sinc_Table[i].imag()*scale);
        j++;
    }
}

/*******************************************************************************
 * Sinc_Read() - Checks for precomputed sinc pulse file, if it exists, it Loads
 *               the content of that file into Sinc_Table and returns true. If
 *               the file does not exist, it returns false.
 *
 * M.Overdick, J.Canfield, and A.G. Klein
 * Last Major Revision: 7/18/2016
 ******************************************************************************/
static bool Sinc_Read(){
    bool success = false;   // Return variable

        // Open file to use for oversampled sinc pulse
    std::ifstream infile("OS_Sinc.dat", std::ifstream::binary);

        // Check if file exists, if so, copy it into array
    if(infile.is_open()){
        while(!infile.eof()){
            infile.read((char*)&Sinc_Table.front(), Sinc_Table.size()*sizeof(CINT16));
        }
        infile.close();
        success = true;
    }else{}

    return success;
}
