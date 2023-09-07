#ifndef NN_QUTEE_CONTROLLER
#define NN_QUTEE_CONTROLLER

// For M_PI constant
#define _USE_MATH_DEFINES

#include <array>
#include <cassert>
#include <cmath>
#include <vector>


#include <eigen3/Eigen/Eigen>
#include <eigen3/unsupported/Eigen/CXX11/Tensor>



template<size_t n_inputs, size_t n_layers, size_t n_neurons, size_t n_outputs>
class NNQuteeController {
public:
    static const int number_weigts = n_neurons* (n_inputs + 1) + n_neurons*(n_neurons+1)* (n_layers - 1) + n_outputs * (n_neurons + 1);

    NNQuteeController():
    _weights(),
    _weights_inputs(_weights.data(),  n_neurons , n_inputs + 1),
    _weights_hidden(_weights.data() + n_neurons * (n_inputs + 1), n_neurons , n_neurons + 1, n_layers - 1 ),
    _weights_ouputs(_weights.data() +  n_neurons * (n_inputs + 1) +  n_neurons*(n_neurons + 1)*(n_layers - 1) , n_outputs ,  n_neurons + 1)
    {
        set_random_weights();
    }
    size_t get_number_weights()
    {
        return number_weigts;
    }

    void set_random_weights()
    {
        _weights.setRandom();
        _weights = (_weights-0.5f)*sqrtf(4.0f * 6.0f/(n_inputs + n_outputs) ); 
         
    }

    void set_weights(const Eigen::TensorFixedSize< float, Eigen::Sizes<NNQuteeController<n_inputs, n_layers, n_neurons, n_outputs>::number_weigts, 1> >& weights)
    {
        _weights = weights;
    }

    Eigen::TensorFixedSize< float, Eigen::Sizes<NNQuteeController<n_inputs, n_layers, n_neurons, n_outputs>::number_weigts, 1> >& get_weights()
    {
       return _weights;
    }


    void set_weights(const Eigen::TensorFixedSize< float , Eigen::Sizes<n_neurons , n_inputs + 1> >& weights_inputs,
                        const Eigen::TensorFixedSize< float , Eigen::Sizes< n_neurons , n_neurons + 1, n_layers - 1> >& weights_hidden,
                        const Eigen::TensorFixedSize< float , Eigen::Sizes<n_outputs ,  n_neurons + 1> >& weights_ouputs)
    {
        _weights_inputs = weights_inputs;
        _weights_hidden = weights_hidden;
        _weights_ouputs = weights_ouputs;
    }



    Eigen::TensorFixedSize< float , Eigen::Sizes<n_outputs, 1> > pos(Eigen::TensorFixedSize< float , Eigen::Sizes<n_inputs, 1> >& inputs) const
    {
        assert(inputs.size() == n_inputs);
        Eigen::array<Eigen::IndexPair<int>, 1> product_dims = { Eigen::IndexPair<int>(1, 0) }; // dimension order for standard matrix multiplication when using tensor  https://eigen.tuxfamily.org/dox/unsupported/eigen_tensors.html#title56 
      
        Eigen::TensorFixedSize< float , Eigen::Sizes<n_inputs + 1, 1> > augmented_inputs; // Extend length of input by one, to add a "1" that is used to define the bias via a weight. (i.e., last_weight*1 = bias)
        augmented_inputs.setConstant(1.0f); // sets everything to 1
        Eigen::array<Eigen::Index, 2> offsets = {0, 0};
        Eigen::array<Eigen::Index, 2> extents = {n_inputs, 1}; 
        augmented_inputs.slice(offsets, extents)=inputs; // sets values using inputs data with the right slice, so last value remains 1.
    
        Eigen::TensorFixedSize< float , Eigen::Sizes<n_neurons + 1, 1> > augmented_intermediate; // define variable
        augmented_intermediate.setConstant(1.0f); // sets everything to 1
        extents = {n_neurons, 1}; 
        augmented_intermediate.slice(offsets, extents) = _weights_inputs.contract(augmented_inputs, product_dims).tanh(); // compute weight*input mat product and then use tanh as activation function. Finally, sets the value of the variable, leaving the last one to 1.
        
        if(n_layers - 1 > 0)
            for(size_t l = 0; l<n_layers - 1 ;l++) // for every hidden layer.
                augmented_intermediate.slice(offsets, extents) = _weights_hidden.chip(l,2).contract(augmented_intermediate, product_dims).tanh(); // We do thre same has before, but we "chip", to slice the corresponding set of weights of the specific layer.
        Eigen::TensorFixedSize< float , Eigen::Sizes<n_outputs, 1> > outputs = _weights_ouputs.contract(augmented_intermediate, product_dims).tanh(); //final output layer. 

        return outputs;
 
    }

    

protected:
    Eigen::TensorFixedSize< float, Eigen::Sizes<NNQuteeController<n_inputs, n_layers, n_neurons, n_outputs>::number_weigts, 1> > _weights;
    Eigen::TensorMap<Eigen::TensorFixedSize< float , Eigen::Sizes<n_neurons , n_inputs + 1> > > _weights_inputs;
    Eigen::TensorMap<Eigen::TensorFixedSize< float , Eigen::Sizes<n_neurons , n_neurons + 1, n_layers - 1> > > _weights_hidden;
    Eigen::TensorMap<Eigen::TensorFixedSize< float , Eigen::Sizes<n_outputs ,  n_neurons + 1> > > _weights_ouputs;

    

};

#endif
