import React, {useState, useEffect} from 'react';
import { Button, View, StyleSheet } from 'react-native';
import CoordinateTest from './CoordinateTest';
import { ScrollView } from 'react-native-gesture-handler';
import axios from 'axios-1.6.8';

function DetailsScreen() {
  const [startButtonText, setStartButtonText] = useState('START');
  const [shutdownButtonText, setShutdownButtonText] = useState('SHUTDOWN');
  const [startButtonColor, setStartButtonColor] = useState('green');
  const [shutdownButtonColor, setShutdownButtonColor] = useState('grey');
  


  const onPressStartHandler = async () => {
    setStartButtonText('STARTING...');

    try {
      // Make a POST request to the server endpoint to start the script
      const response = await axios.post('http://192.168.2.55\:3000/start-script');
      console.log(response.data); // Log the response from the server
      
    } catch (error) {
      console.error('Error:', error.response ? error.response.data : error.message);
      setStartButtonText('START');
      setStartButtonColor(null);
    }
    setStartButtonText('STARTED')
    setStartButtonColor('green');
  };

  const onPressShutdownHandler = async () => {
    setShutdownButtonText('SHUTTING DOWN...');
    setShutdownButtonColor('red');

    try {
      // Make a POST request to the server endpoint to start the script
      const response = await axios.post('http://192.168.2.55\:3000/end-script');
      console.log(response.data); // Log the response from the server
      
    } catch (error) {
      console.error('Error:', error.response ? error.response.data : error.message);
      setStartButtonText('SHUTDOWN');
      setStartButtonColor(null);
    }
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
  }
});

export default DetailsScreen;