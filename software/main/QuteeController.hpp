#ifndef QUTEE_CONTROLLER
#define QUTEE_CONTROLLER

// For M_PI constant
#define _USE_MATH_DEFINES

#include <array>
#include <cassert>
#include <cmath>
#include <vector>

#include <eigen3/Eigen/Eigen>


#define ARRAY_DIM 100
typedef Eigen::Matrix< float , Eigen::Dynamic , 1> VectorXf;
class QuteeController {
public:
    QuteeController() {}
    QuteeController(const VectorXf& ctrl)
    {
        set_parameters(ctrl);
    }

    void set_parameters(const VectorXf& ctrl)
    {
        assert(ctrl.size() == 24);

        _legs0commands0 = VectorXf(ARRAY_DIM);
        _legs1commands0 = VectorXf(ARRAY_DIM);
        _legs2commands0 = VectorXf(ARRAY_DIM);
        _legs3commands0 = VectorXf(ARRAY_DIM);
        _legs0commands1 = VectorXf(ARRAY_DIM);
        _legs1commands1 = VectorXf(ARRAY_DIM);
        _legs2commands1 = VectorXf(ARRAY_DIM);
        _legs3commands1 = VectorXf(ARRAY_DIM);
        _legs0commands2 = VectorXf(ARRAY_DIM);
        _legs1commands2 = VectorXf(ARRAY_DIM);
        _legs2commands2 = VectorXf(ARRAY_DIM);
        _legs3commands2 = VectorXf(ARRAY_DIM);
      
        _params = ctrl;

        _legs0commands0 = _control_signal(ctrl[0], ctrl[1], ctrl[2]);
        _legs0commands1 = _control_signal(ctrl[3], ctrl[4], ctrl[5]);
        _legs0commands2 = _control_signal(ctrl[3], ctrl[4], ctrl[5]);

        _legs1commands0 = _control_signal(ctrl[6], ctrl[7], ctrl[8]);
        _legs1commands1 = _control_signal(ctrl[9], ctrl[10], ctrl[11]);
        _legs1commands2 = _control_signal(ctrl[9], ctrl[10], ctrl[11]);

        _legs2commands0 = _control_signal(ctrl[12], ctrl[13], ctrl[14]);
        _legs2commands1 = _control_signal(ctrl[15], ctrl[16], ctrl[17]);
        _legs2commands2 = _control_signal(ctrl[15], ctrl[16], ctrl[17]);

        _legs3commands0 = _control_signal(ctrl[18], ctrl[19], ctrl[20]);
        _legs3commands1 = _control_signal(ctrl[21], ctrl[22], ctrl[23]);
        _legs3commands2 = _control_signal(ctrl[21], ctrl[22], ctrl[23]);
    }

    const VectorXf& parameters() const
    {
        return _params;
    }

    VectorXf pos(double t) const
    {
        assert(_params.size() == 24);

        VectorXf angles(12);
        size_t t_index = (int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM;
        // leg 0
        angles(0)  =  M_PI_4 * _legs0commands0[t_index];
        angles(1)  =  M_PI_4 * _legs0commands1[t_index];
        angles(2)  = -M_PI_4 * _legs0commands2[t_index];
        // leg 1    
        angles(3)  =  M_PI_4 * _legs1commands0[t_index];
        angles(4)  =  M_PI_4 * _legs1commands1[t_index];
        angles(5)  = -M_PI_4 * _legs1commands2[t_index];
        // leg 2
        angles(6)  =  M_PI_4 * _legs2commands0[t_index];
        angles(7)  =  M_PI_4 * _legs2commands1[t_index];
        angles(8)  = -M_PI_4 * _legs2commands2[t_index];
        // leg 3
        angles(9)  =  M_PI_4 * _legs3commands0[t_index];
        angles(10) =  M_PI_4 * _legs3commands1[t_index];
        angles(11) = -M_PI_4 * _legs3commands2[t_index];

        return angles;
    }

protected:
    /**
        All parameters should have a value between 0 and 1.
    **/
    VectorXf _control_signal(float amplitude, float phase, float duty_cycle) const
    {
        
        VectorXf temp(ARRAY_DIM);
        int up_time = ARRAY_DIM * duty_cycle;
        for (int i = 0; i < up_time; i++)
            temp[i] = amplitude;
        for (int i = up_time; i < ARRAY_DIM; i++)
            temp[i] = -amplitude;

        // filtering
        int kernel_size = ARRAY_DIM / 10;

        VectorXf  command(ARRAY_DIM);

        std::vector<double> kernel(2 * kernel_size + 1, 0);
        double sigma = kernel_size / 3;

        double sum = 0;
        for (int i = 0; i < int(kernel.size()); i++) {
            kernel[i] = exp(-(i - kernel_size) * (i - kernel_size) / (2 * sigma * sigma)) / (sigma * sqrt(M_PI));
            sum += kernel[i];
        }

        for (int i = 0; i < ARRAY_DIM; i++) {
            command[i] = 0;
            for (int d = 1; d <= kernel_size; d++) {
                if (i - d < 0)
                    command[i] += temp[ARRAY_DIM + i - d] * kernel[kernel_size - d];
                else
                    command[i] += temp[i - d] * kernel[kernel_size - d];
            }
            command[i] += temp[i] * kernel[kernel_size];
            for (int d = 1; d <= kernel_size; d++) {
                if (i + d >= ARRAY_DIM)
                    command[i] += temp[i + d - ARRAY_DIM] * kernel[kernel_size + d];
                else
                    command[i] += temp[i + d] * kernel[kernel_size + d];
            }

            command[i] /= sum;
        }

        // apply phase
        VectorXf final_command;
        int current = 0;
        int start = std::floor(ARRAY_DIM * phase);
        for (int i = start; i < ARRAY_DIM; i++) {
            final_command[current] = command[i];
            current++;
        }
        for (int i = 0; i < start; i++) {
            final_command[current] = command[i];
            current++;
        }

        return final_command;
    }

    VectorXf _legs0commands0;
    VectorXf _legs1commands0;
    VectorXf _legs2commands0;
    VectorXf _legs3commands0;
    VectorXf _legs0commands1;
    VectorXf _legs1commands1;
    VectorXf _legs2commands1;
    VectorXf _legs3commands1;
    VectorXf _legs0commands2;
    VectorXf _legs1commands2;
    VectorXf _legs2commands2;
    VectorXf _legs3commands2;


    VectorXf _params;

};

#endif
