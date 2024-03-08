import React, {useState} from 'react';
import { Button, View } from 'react-native';
import CoordinateTest from './CoordinateTest';

function DetailsScreen() {
    
  const [text, setText] = useState("SHUTDOWN");
  const [buttonColor, setButtonColor] = useState();
 


  const onPressHandler = () => {
    setText("CONFIRM SHUTDOWN");
    setButtonColor("red");
  };

  return (
    <View>
      <View style={{paddingTop: 0}}>
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

export default DetailsScreen;