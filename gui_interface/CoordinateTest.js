import React, { useState, useEffect } from 'react';
import { View, Text, StyleSheet } from 'react-native';
import Roslib from 'roslib';

const CoordinateTest = () => {
  const [points1, setPoints1] = useState([]);
  const [points2, setPoints2] = useState([]);
  const [points3, setPoints3] = useState([]);
  let pointCounter = 0;

  const renderGrid = () => {
    const grid = [];
    for (let row = 0; row < 36; row++) {
        const rowNodes = [];
        for (let col = 0; col < 30; col++) {
          rowNodes.push(<Node key={`node-${row}-${col}`} />);
        }
      grid.push(<View key={`row-${row}`} style={styles.row}>{rowNodes}</View>);
    }
    return grid;
  };
  useEffect(() => {
    const ros = new Roslib.Ros({
      url: 'ws://192.168.246.130:9090'
    });

    const topicListener1 = new Roslib.Topic({
      ros: ros,
      name: '/obstacle_csv_topic',
      messageType: 'interfaces/Obstacles'
    });

    const topicListener2 = new Roslib.Topic({
      ros: ros,
      name: '/checkpoints_csv_topic',
      messageType: 'interfaces/Checkpoints'
    });

    const topicListener3 = new Roslib.Topic({
      ros: ros,
      name: '/environment_csv_topic',
      messageType: 'interfaces/Environment'
    });

    const subscribeToRosTopic1 = (message) => {
      // Parse the message
      const newPoint = {
        key: pointCounter++,
        easting: message.easting,
        northing: message.northing,
        elevation: message.elevation,
        bounding_radius: message.bounding_radius,
      };
      setPoints1(prevPoints => [...prevPoints, newPoint]);
    };

    const subscribeToRosTopic2 = (message) => {
      const newPoint = {
        key: pointCounter++,
        easting: message.easting,
        northing: message.northing,
      };
      setPoints2(prevPoints => [...prevPoints, newPoint]);
    };

    const subscribeToRosTopic3 = (message) => {
      const newPoint = {
        key: pointCounter++,
        easting: message.easting,
        northing: message.northing,
      };
      setPoints3(prevPoints => [...prevPoints, newPoint]);
    };

    topicListener1.subscribe(subscribeToRosTopic1);
    topicListener2.subscribe(subscribeToRosTopic2);
    topicListener3.subscribe(subscribeToRosTopic3);

    return () => {
      topicListener1.unsubscribe();
      topicListener2.unsubscribe();
      topicListener3.unsubscribe();
      ros.close();
    };
  }, []);

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
      left: ((x * 12) + 183.5) - size / 2,
      top: (423 - (y * 12)) - size / 2,
      fontSize: size,
      color: color,
    };
    return <Text style={textStyle}>{text}</Text>;
  };

  return (
      <View style={styles.gridContainer}>
        {renderGrid()}
        <TextComponent text="0" size={16} x={0} y={-2} color="black" />
        <TextComponent text="5" size={16} x={5} y={-2} color="black" />
        <TextComponent text="10" size={16} x={9.5} y={-2} color="black" />
        <TextComponent text="15" size={16} x={14.5} y={-2} color="black" />
        <TextComponent text="-5" size={16} x={-5.4} y={-2} color="black" />
        <TextComponent text="-10" size={16} x={-10.7} y={-2} color="black" />
        <TextComponent text="-15" size={16} x={-15.7} y={-2} color="black" />
        <TextComponent text="5" size={16} x={0} y={4.5} color="black" />
        <TextComponent text="10" size={16} x={-0.3} y={9.5} color="black" />
        <TextComponent text="15" size={16} x={-0.3} y={14.5} color="black" />
        <TextComponent text="20" size={16} x={-0.3} y={19.5} color="black" />
        <TextComponent text="25" size={16} x={-0.3} y={24.5} color="black" />
        <TextComponent text="30" size={16} x={-0.3} y={29.5} color="black" />
        <TextComponent text="35" size={16} x={-0.3} y={34.5} color="black" />
        <XAxis x={0} y={432} length={359} color="black" />
        <YAxis x={179} y={12} height={432} color="black" />
        {points1.map((point1, index) => (
          <Point key={`point1-${index}`} size={8 * point1.bounding_radius} x={point1.easting} y={point1.northing} color="red" />
        ))}
        {points2.map((point2, index) => (
          <Point key={`point2-${index}`} size={10} x={point2.easting} y={point2.northing} color="green" />
        ))}
        {points3.map((point3, index) => (
          <Point key={`point3-${index}`} size={8} x={point3.easting} y={point3.northing} color="purple" />
        ))}
      </View>  
  );
};





const Node = () => {
    return (
      <View style={styles.node} />
    );
  };

  

  const Point = ({ size, x, y, color }) => {
    const pointStyle = {
      position: 'absolute',
      left: ((x * 12) + 180) - size / 2,
      top: (432 - (y * 12)) - size / 2,
      width: size,
      height: size,
      backgroundColor: color, // You can customize the color here
      borderRadius: size / 2,
    };
  
  return <View style={pointStyle} />;
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

export default CoordinateTest;