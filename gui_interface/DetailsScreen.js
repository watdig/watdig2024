import React, {useState} from 'react';
import { Button, View, StyleSheet } from 'react-native';
import CoordinateTest from './CoordinateTest';
import { ScrollView } from 'react-native-gesture-handler';

function DetailsScreen() {
    
  const [text, setText] = useState("SHUTDOWN");
  const [buttonColor, setButtonColor] = useState();
 


  const onPressHandler = () => {
    setText("CONFIRM SHUTDOWN");
    setButtonColor("red");
  };

  return (
    <ScrollView contentContainerStyle={styles.scrollView}>
      <ScrollView horizontal>
        <View style={styles.container}>
          <View>
            <Button
              title='START'
              onPress={() => console.log('START button pressed')}
            />
          </View>
          <View>
            <Button
              title={text}
              onPress={onPressHandler}
              color={buttonColor}
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