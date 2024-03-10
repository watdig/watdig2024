import React, {useState} from 'react';
import { Button, View } from 'react-native';
import CoordinateGrid from './CoordinateGrid';

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
        <CoordinateGrid />
      </View>
    </View>
    );
}

export default DetailsScreen;