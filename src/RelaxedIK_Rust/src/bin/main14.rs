extern crate neuroflow;

use neuroflow::FeedForward;
use neuroflow::data::DataSet;
use neuroflow::activators::Type::Tanh;
use std::time::{Instant, Duration};


fn main(){
    /*
        Define neural network with 1 neuron in input layers. Network contains 4 hidden layers.
        And, such as our function returns single value, it is reasonable to have 1 neuron in the output layer.
    */
    let mut nn = FeedForward::new(&[1, 30, 30, 30, 30, 30, 1]);

    /*
        Define DataSet.

        DataSet is the Type that significantly simplifies work with neural network.
        Majority of its functionality is still under development :(
    */
    let mut data: DataSet = DataSet::new();
    let mut i = -3.0;

    // Push the data to DataSet (method push accepts two slices: input data and expected output)
    while i <= 2.5 {
        data.push(&[i], &[0.5*(i.exp().sin()) - (-i.exp()).cos()]);
        i += 0.05;
    }

    // Here, we set necessary parameters and train neural network by our DataSet with 50 000 iterations
    nn.activation(Tanh)
        .learning_rate(0.01)
        .train(&data, 50_000);

    let mut res;

    // Let's check the result
    i = 0.0;
    while i <= 0.3{
        res = nn.calc(&[i])[0];
        println!("for [{:.3}], [{:.3}] -> [{:.3}]", i, 0.5*(i.exp().sin()) - (-i.exp()).cos(), res);
        i += 0.07;
    }

    let mut res = nn.calc(&[0.0])[0];
    let start = Instant::now();
    for i in 0..1000 {
        res = nn.calc(&[0.0])[0];
    }
    let duration = start.elapsed();
    println!("{:?}", res);
    println!("{:?}", duration);

}