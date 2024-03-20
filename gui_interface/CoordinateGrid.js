import React from 'react';
import { View, Text, StyleSheet } from 'react-native';


const CoordinateGrid = () => {
  const XAxis = ({ x, y, length, color }) => {
    const lineStyle = {
      position: 'absolute',
      left: x,
      top: y,
      width: length,
      height: 2, // Adjust the height of the line as needed
      backgroundColor: color,
    };
  
    return <View style={lineStyle} />;
  };

  const YAxis = ({ x, y, height, color }) => {
    const lineStyle = {
      position: 'absolute',
      left: x,
      top: y,
      width: 2, // Adjust the width of the line as needed
      height: height,
      backgroundColor: color,
    };
  
    return <View style={lineStyle} />;
  };
  const TextComponent = ({ text, size, x, y, color }) => {
    const textStyle = {
      position: 'absolute',
      left: ((x * 12) + 303.5) - size / 2,
      top: (483 - (y * 12)) - size / 2,
      fontSize: size,
      color: color,
    };
    return <Text style={textStyle}>{text}</Text>;
  };
  const renderGrid = () => {
    const grid = [];
    for (let row = 0; row < 80; row++) {
        const rowNodes = [];
      for (let col = 0; col < 50; col++) {
        rowNodes.push(<Node key={`node-${row}-${col}`} />);
      }
      grid.push(<View key={`row-${row}`} style={styles.row}>{rowNodes}</View>);
    }
    return grid;
  };

  return (
    <View style={styles.gridContainer}>
      {renderGrid()}
      <TextComponent text="0" size={16} x={0} y={-0.45} color="black" />
      <TextComponent text="5" size={16} x={5} y={-2} color="black" />
      <TextComponent text="10" size={16} x={9.5} y={-2} color="black" />
      <TextComponent text="15" size={16} x={14.5} y={-2} color="black" />
      <TextComponent text="20" size={16} x={19.5} y={-2} color="black" />
      <TextComponent text="-5" size={16} x={-5.4} y={-2} color="black" />
      <TextComponent text="-10" size={16} x={-10.7} y={-2} color="black" />
      <TextComponent text="-15" size={16} x={-15.7} y={-2} color="black" />
      <TextComponent text="-20" size={16} x={-20.7} y={-2} color="black" />
      <TextComponent text="5" size={16} x={0} y={4.5} color="black" />
      <TextComponent text="10" size={16} x={-0.3} y={9.5} color="black" />
      <TextComponent text="15" size={16} x={-0.3} y={14.5} color="black" />
      <TextComponent text="20" size={16} x={-0.3} y={19.5} color="black" />
      <TextComponent text="25" size={16} x={-0.3} y={24.5} color="black" />
      <TextComponent text="30" size={16} x={-0.3} y={29.5} color="black" />
      <TextComponent text="35" size={16} x={-0.3} y={34.5} color="black" />
      <TextComponent text="-5" size={16} x={-0.3} y={-5.5} color="black" />
      <TextComponent text="-10" size={16} x={-0.7} y={-10.5} color="black" />
      <TextComponent text="-15" size={16} x={-0.7} y={-15.5} color="black" />
      <TextComponent text="-20" size={16} x={-0.7} y={-20.5} color="black" />
      <TextComponent text="-25" size={16} x={-0.7} y={-25.5} color="black" />
      <TextComponent text="-30" size={16} x={-0.7} y={-30.5} color="black" />
      <TextComponent text="-35" size={16} x={-0.7} y={-35.5} color="black" />
      <XAxis x={0} y={491.5} length={600} color="black" />
      <YAxis x={299} y={12} height={960} color="black" />
      <Point size={10} x={0} y={0} />
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
      left: ((x * 12) + 300) - size / 2,
      top: (492 - (y * 12)) - size / 2,
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
      left: ((x * 18) + x + 0) - size / 2,
      top: (0 - (y * 18) + y) - size / 2,
      width: size,
      height: size,
      backgroundColor: 'red', // You can customize the color here
      borderRadius: size / 2,
    };   
  
    return <View style={point2Style} />;
  };
  
  

const styles = StyleSheet.create({
  gridContainer: {
    paddingTop: 12,
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