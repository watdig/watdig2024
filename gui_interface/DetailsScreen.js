import React, {useState, useEffect} from 'react';
import { Button, View, StyleSheet, Text } from 'react-native';
import CoordinateTest from './CoordinateTest';
import { ScrollView } from 'react-native-gesture-handler';
import Roslib from 'roslib';
//import axios from 'axios-1.6.8';


function DetailsScreen() {
  const [startButtonText, setStartButtonText] = useState('START');
  const [shutdownButtonText, setShutdownButtonText] = useState('SHUTDOWN');
  const [startButtonColor, setStartButtonColor] = useState('green');
  const [shutdownButtonColor, setShutdownButtonColor] = useState('grey');
  const [containerColor, setContainerColor] = useState('green');
  const [container2Color, setContainer2Color] = useState('red');
  const [gyroValue, setGyroValue] = useState(null);

  useEffect(() => {
    const ros = new Roslib.Ros({
      url: 'ws://172.20.10.14:9090'
    });

    const gyroClient = new Roslib.Service({
      ros: ros,
      name: '/gyro_service',
      serviceType: 'interfaces/Gyroserv'
    });

    const gyroRequest = new Roslib.ServiceRequest();

      const handleGyroResponse = (response) => {
        setGyroValue(response.value);
        setContainer2Color('green'); // Update container color when service call is successful
    };

    gyroClient.callService(gyroRequest, handleGyroResponse);

    // Set container color to green when component is unmounted


    ros.on('connection', () => {
      setContainerColor('red');
    });

    ros.on('error', () => {
      setContainerColor('green');
      setContainer2Color('red');
    });

    return () => {
      ros.close();
      
    };
  }, []);




  const onPressStartHandler = async () => {
    setStartButtonText('STARTING...');

    /* try {
      // Make a POST request to the server endpoint to start the script
      const response = await axios.post('http://172.20.10.7\:3000/start-script');
      console.log(response.data); // Log the response from the server
      
    } catch (error) {
      console.error('Error:', error.response ? error.response.data : error.message);
      setStartButtonText('START');
      setStartButtonColor(null);
    }
    setStartButtonText('STARTED')
    setStartButtonColor('green');
*/
  }; 

  const onPressShutdownHandler = async () => {
    setShutdownButtonText('SHUTTING DOWN...');
    setShutdownButtonColor('red');

    /* try {
      // Make a POST request to the server endpoint to start the script
      const response = await axios.post('http://172.20.10.7\:3000/end-script');
      console.log(response.data); // Log the response from the server
      
    } catch (error) {
      console.error('Error:', error.response ? error.response.data : error.message);
      setStartButtonText('SHUTDOWN');
      setStartButtonColor(null);
    } */
  };


  return (
    <ScrollView contentContainerStyle={styles.scrollView}>
      <ScrollView horizontal>
        <View style={styles.container}>
          <View>
            <Button
              title={startButtonText}
              onPress={onPressStartHandler}
              color={startButtonColor}
            />
          </View>
          <View>
            <Button
              title={shutdownButtonText}
              onPress={onPressShutdownHandler}
              color={shutdownButtonColor}
            />
          </View>
          <View>
            <CoordinateTest />
          </View>
          <View>
          <View style={[styles.statusContainer, { backgroundColor: containerColor }]}>
              <Text style={styles.statusText}>Machine Status</Text>
          </View>
          <View style={[styles.statusContainer, { backgroundColor: container2Color }]}>
            <Text style={styles.statusText}>Gyro Status</Text>
            <Text style={styles.statusText}>{gyroValue}</Text>
      </View>
          </View>
        </View>
      </ScrollView>
    </ScrollView>
  );
}

const styles = StyleSheet.create({
  scrollView: {
    flexGrow: 1,
  },
  container: {
    justifyContent: 'center',
    alignItems: 'center'
  },
  statusContainer: {
    backgroundColor: 'green',
    padding: 10,
    marginTop: 20,
    borderRadius: 5,
    alignSelf: 'center'
  },
  statusText: {
    color: 'white',
    fontWeight: 'bold',
  }
});

export default DetailsScreen;