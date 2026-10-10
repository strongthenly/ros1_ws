#ifndef __DIGITAL_FILTER_H__
#define __DIGITAL_FILTER_H__

typedef struct
{
 double Input_Butter[3];
 double Output_Butter[3];
}Butter_BufferData;


typedef struct
{
  double a[3];
  double b[3];
}Butter_Parameter;

class digital_filter
{
    public:
        digital_filter();
        ~digital_filter();

        Butter_Parameter Accel_Parameter;
        Butter_Parameter Gyro_Parameter;
        Butter_BufferData  accel_filter_buf[3];
        Butter_BufferData  gyro_filter_buf[3];

        // Functions
        void Set_Cutoff_Frequency(double sample_frequent,double cutoff_frequent,Butter_Parameter *LPF);
        double LPButterworth(double curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter);

};

#endif

