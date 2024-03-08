import React from 'react';
import { View, StyleSheet } from 'react-native';


const CoordinateGrid = () => {
    
  const renderGrid = () => {
    const grid = [];
    for (let row = 0; row < 20; row++) {
        const rowNodes = [];
      for (let col = 0; col < 20; col++) {
        rowNodes.push(<Node key={`node-${row}-${col}`} />);
      }
      grid.push(<View key={`row-${row}`} style={styles.row}>{rowNodes}</View>);
    }
    return grid;
  };

  return (
    <View style={styles.gridContainer}>
      {renderGrid()}
      <Point size={10} x={10} y={1} />
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
      left: ((x * 18) + x + 7) - size / 2,
      top: (378 - (y * 18) + y) - size / 2,
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
      top: (378 - (y * 18) + y) - size / 2,
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
    width: 17,
    height: 17,
    backgroundColor: 'white',
    margin: 1,
  },
});

export default CoordinateGrid;