/*******************************************************************************
 * sinc.hpp - Modulated sinc pulse generator header.
 *
 * M.Overdick, J.Canfield, and A.G.Klein
 * Last Major Revision: 7/14/2016
 ******************************************************************************/

#ifndef SINC_HPP
#define SINC_HPP

    // Initializes oversampled sinc pulse
void Sinc_Init(FP32 bw, FP32 cbw, INT32U spb, INT16U ratio);

    // Creates template sinc pulse for cross correlation
void Sinc_Gen_XC(CINT64 * table, INT64 ampl, INT16U spb);

    // Undersamples and creates centered pulse on I channel and a delayed pulse
    // on Q channel.
void Sinc_Gen_TX(CINT16 * table, INT16U ampl, INT16U spb, FP32 tau, bool I_en);

    // Creates template sinc pulse for cross correlation with Cuda
void Cuda_Sinc_Gen_XC(FP32 * table_real, FP32 * table_imag, INT64 ampl, INT16U spb);


#else
#endif
