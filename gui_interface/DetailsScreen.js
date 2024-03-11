import React, {useState} from 'react';
import { Button, View, StyleSheet } from 'react-native';
import CoordinateTest from './CoordinateTest';

function DetailsScreen() {
    
  const [text, setText] = useState("SHUTDOWN");
  const [buttonColor, setButtonColor] = useState();
 


  const onPressHandler = () => {
    setText("CONFIRM SHUTDOWN");
    setButtonColor("red");
  };

  return (
    <View style={styles.container}>
      <View>
        <Button
          title='START'
        />
      </View>
      <View>
        <Button title={text} onPress={onPressHandler} color={buttonColor} />
      </View>
      <View>
        <CoordinateTest />
      </View>
    </View>
    );
}
const styles = StyleSheet.create({
  container: {
    justifyContent: 'center',
    alignItems: 'center'
  }
});
export default DetailsScreen;