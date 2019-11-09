/*
    Receiver.h - Library for taking inputs from a radio-controlled receiver
    Created by olive, November 8, 2019
*/

#ifndef Receiver_h
#define Receiver_h

class Receiver
{
    public:
        Receiver(int[] channelPins);
        void init();
        void setCalibValues();
        int[][] calibValues();
        

    private:
        int[] _channelPins;
};

#endif