import React from 'react';
import { View, Text, StyleSheet } from 'react-native';


const CoordinateGrid = () => {
    
  const renderGrid = () => {
    const grid = [];
    for (let row = 0; row < 48; row++) {
        const rowNodes = [];
      for (let col = 0; col < 100; col++) {
        rowNodes.push(<Node key={`node-${row}-${col}`} />);
      }
      grid.push(<View key={`row-${row}`} style={styles.row}>{rowNodes}</View>);
    }
    return grid;
  };

  return (
    <View style={styles.gridContainer}>
      {renderGrid()}
      <Point size={8} x={0} y={0} />
      <Point2 size={10} x={11} y={1} />
    </View>
  );
};

const Node = () => {
  return (
    <View style={styles.node} />
  );
};

const Point = ({ size, x, y }) => {
    const pointStyle = {
      position: 'absolute',
      left: ((x * 10) + x + 769) - size / 2,
      top: (407 - (y * 10) - y) - size / 2,
      width: size,
      height: size,
      backgroundColor: 'green', // You can customize the color here
      borderRadius: size / 2,
    };
  
  return <View style={pointStyle} />;
};
  
  const Point2 = ({ size, x, y }) => {
    const point2Style = {
      position: 'absolute',
      left: ((x * 18) + x + 7) - size / 2,
      top: (396 - (y * 18) + y) - size / 2,
      width: size,
      height: size,
      backgroundColor: 'red', // You can customize the color here
      borderRadius: size / 2,
    };   
  
    return <View style={point2Style} />;
  };
  
  

const styles = StyleSheet.create({
  gridContainer: {
    flexDirection: 'column',
    justifyContent: 'center',
    alignItems: 'center',
  },
  row: {
    flexDirection: 'row',
  },
  node: {
    width: 10,
    height: 10,
    backgroundColor: 'white',
    margin: 1,
  },
});

export default CoordinateGrid;