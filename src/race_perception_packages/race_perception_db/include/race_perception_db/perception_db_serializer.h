#ifndef _PERCEPTION_DB_SERIALIZER_H_
#define _PERCEPTION_DB_SERIALIZER_H_

#include <iostream>
#include <string>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <ros/ros.h>

using namespace std;

namespace race_perception_db
{

template <typename B, typename T>
struct PerceptionDBSerializer
{

    static void serialize(B& buffer, const T& t, uint32_t i);
    
    //template<typename T, typename Stream>
    static void deserialize(B& buffer, T& t, uint32_t i);
    
}; //struct PerceptionDBSerializer


template <typename B, typename T>
void PerceptionDBSerializer<B, T>::serialize(B& buffer, const T& t, uint32_t i)
{
    //Compute msg_in size
    //uint32_t serial_size = ros::serialization::serializationLength(t);
    uint32_t serial_size = i;

	//Declare a shared array
	//boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);

	//Declare a stream
	ros::serialization::OStream stream(buffer.get(), serial_size);
	//memcpy(stream.getData(), buffer.get(), serial_size);

	//Serialize msg_in
	ros::serialization::serialize(stream, t);
}

template <typename B, typename T>
void PerceptionDBSerializer<B, T>::deserialize(B& buffer, T& t, uint32_t i)
{
    //Compute msg_in size
    uint32_t deserial_size = i;

	//memcpy(buffer1.get(), value.data(), value.size());

	//Declare an Input stream
	ros::serialization::IStream stream_des(buffer.get(), deserial_size); //Get a istream from buffer

	//Deserialize istream to msg_out
	ros::serialization::Serializer<T>::read(stream_des, t);
}
    
}; //namespace race_perception_db

#endif
