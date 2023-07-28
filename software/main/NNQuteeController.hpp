#ifndef NN_QUTEE_CONTROLLER
#define NN_QUTEE_CONTROLLER

// For M_PI constant
#define _USE_MATH_DEFINES

#include <array>
#include <cassert>
#include <cmath>
#include <vector>

#include <eigen3/Eigen/Eigen>

template<size_t n_inputs, size_t n_layers, size_t n_neurons, size_t n_outputs>
class NNQuteeController {
public:
    NNQuteeController() {
        set_random_weights();
    }
    size_t get_number_params()
    {
        return n_neurons* (n_inputs + 1) + n_neurons*(n_neurons+1)* (n_layers - 1) + n_outputs * (n_neurons + 1);
    }

    //TODO CREATE FUNCTION TO SERIALISIE (put as a single vector) all the weights. As get and set. 

    void set_random_weights()
    {
        //TODO: REPLACE THAT WITH XAVIER INIT.
        _weights_inputs =  Eigen::Matrix< float , n_neurons , n_inputs + 1>::Zero();
        _weights_hidden  = Eigen::Matrix< float , n_neurons , n_neurons + 1, n_layers - 1 >::Zero();
        _weights_ouputs =  Eigen::Matrix< float , n_outputs ,  n_neurons + 1>::Zero();
    }

    void set_parameters(const Eigen::Matrix< float , n_neurons , n_inputs + 1>& weights_inputs,
                        const Eigen::Matrix< float , n_neurons , n_neurons + 1, n_layers - 1 >& weights_hidden,
                        const Eigen::Matrix< float , n_outputs ,  n_neurons + 1>& weights_ouputs)
    {
        _weights_inputs = weights_inputs;
        _weights_hidden = weights_hidden;
        _weights_ouputs = weights_ouputs;
    }


    Eigen::Matrix< float , n_outputs , 1> pos(Eigen::Matrix< float , n_inputs , 1> inputs) const
    {
        assert(inputs.size() == n_inputs);
        Eigen::Matrix< float , n_inputs + 1, 1> augmented_inputs;
        augmented_inputs <<  inputs, 1;
        Eigen::Matrix< float , n_neurons + 1, 1> augmented_intermediate;
        augmented_intermediate << (_weights_inputs * augmented_inputs).array().tanh(), 1;
        if(n_layers - 1 >0)
            for(size_t l = 0; l<n_layers - 1 ;l++)
                augmented_intermediate << (_weights_hidden * augmented_intermediate).array().tanh(),1;
        Eigen::Matrix< float , n_outputs, 1> outputs = (_weights_ouputs * augmented_intermediate).array().tanh();
        return outputs;
    }

protected:
    Eigen::Matrix< float , n_neurons , n_inputs + 1> _weights_inputs;
    Eigen::Matrix< float , n_neurons , n_neurons + 1, n_layers - 1 > _weights_hidden;
    Eigen::Matrix< float , n_outputs ,  n_neurons + 1> _weights_ouputs;

    

};

#endif
