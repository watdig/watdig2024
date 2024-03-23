import React, {useState, useEffect} from 'react';
import { Button, View, StyleSheet } from 'react-native';
import CoordinateTest from './CoordinateTest';
import { ScrollView } from 'react-native-gesture-handler';
import axios from 'axios-1.6.8'; // Install axios using npm or yarn

function DetailsScreen() {
  const [startButtonText, setStartButtonText] = useState('START');
  const [shutdownButtonText, setShutdownButtonText] = useState('SHUTDOWN');
  const [startButtonColor, setStartButtonColor] = useState();
  const [shutdownButtonColor, setShutdownButtonColor] = useState();
  

  const onPressStartHandler = async () => {
    setStartButtonText('STARTING...');
    setStartButtonColor('green');

    try {
      const response = await axios.post('http://localhost:3000/start-server');

      if (response.status === 200) {
        console.log('Command executed successfully');
      } else {
        console.error('Failed to execute command');
      }
    } catch (error) {
      console.error('Error:', error.message);
    }
  };





  

  const onPressShutdownHandler = async () => {
    setShutdownButtonText('SHUTDOWN CONFIRMED');
    setShutdownButtonColor('red');
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