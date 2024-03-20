import React from 'react';
import { Button, View, Text } from 'react-native';

function HomeScreen({ navigation }) {
  return (
    <View style={{ flex: 1, alignItems: 'center', justifyContent: 'center' }}>
      <Text>Welcome to the WatDig Interface</Text>
      <Button
        title="Enter"
        onPress={() => navigation.navigate('Interface')}
      />
    </View>
  );
}

export default HomeScreen;