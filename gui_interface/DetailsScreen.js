import React, {useState, useEffect} from 'react';
import { Button, View, StyleSheet } from 'react-native';
import CoordinateTest from './CoordinateTest';
import { ScrollView } from 'react-native-gesture-handler';
import ROSLIB from 'roslib';

function DetailsScreen() {
    
  const [startButtonText, setStartButtonText] = useState('START');
  const [shutdownButtonText, setShutdownButtonText] = useState('SHUTDOWN');
  const [startButtonColor, setStartButtonColor] = useState();
  const [shutdownButtonColor, setShutdownButtonColor] = useState();
  // const [ros, setRos] = useState(null);
  // const [startPublisher, setStartPublisher] = useState(null);
  // const [shutdownPublisher, setShutdownPublisher] = useState(null);
  // const [startPublishedValue, setStartPublishedValue] = useState(false);
  // const [shutdownPublishedValue, setShutdownPublishedValue] = useState(true);
 
  /* useEffect(() => {
    const rosInstance = new ROSLIB.Ros({
      url: 'ws://192.168.246.130:9090'
    });
    setRos(rosInstance);

    const startPublisherInstance = new ROSLIB.Topic({
      ros: rosInstance,
      name: '/start_topic', 
      messageType: 'interfaces/Start'
    });
    setStartPublisher(startPublisherInstance);

    const shutdownPublisherInstance = new ROSLIB.Topic({
      ros: rosInstance,
      name: '/shutdown_topic', 
      messageType: 'interfaces/Shutdown'
    });
    setShutdownPublisher(shutdownPublisherInstance);

    if (startPublisher && shutdownPublisher) {
      const startMessage = new ROSLIB.Message({
        data: false
      });
      startPublisher.publish(startMessage);
      setStartPublishedValue(false);

      const shutdownMessage = new ROSLIB.Message({
        data: true
      });
      shutdownPublisher.publish(shutdownMessage);
      setShutdownPublishedValue(true);
    }

    return () => {
      rosInstance.close();
    };
  }, []); */

  const onPressStartHandler = () => {
    setStartButtonText('START');
    setStartButtonColor('green');
    /* if (startPublisher && !startPublishedValue) {
      const message = new ROSLIB.Message({
        data: true
      });
      startPublisher.publish(message);
      setStartPublishedValue(true);
    } */
  };

  const onPressShutdownHandler = () => {
    setShutdownButtonText('CONFIRM SHUTDOWN');
    setShutdownButtonColor('red');
  /*   if (shutdownPublisher && shutdownPublishedValue) {
      const message = new ROSLIB.Message({
        data: false
      });
      shutdownPublisher.publish(message);
      setShutdownPublishedValue(false);
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
            //  disabled={startPublishedValue}
            />
          </View>
          <View>
            <Button
              title={shutdownButtonText}
              onPress={onPressShutdownHandler}
              color={shutdownButtonColor}
              // disabled={!shutdownPublishedValue}
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