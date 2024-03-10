import React, { useState, useEffect } from 'react';
import { View, StyleSheet } from 'react-native';
import Roslib from 'roslib';

const CoordinateTest = () => {
  const [points1, setPoints1] = useState([]);
  const [points2, setPoints2] = useState([]);
  const [points3, setPoints3] = useState([]);
  let pointCounter = 0;

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

  return (
    <View>
      <View style={styles.gridContainer}>
        {renderGrid()}
        <XAxis x={169} y={408} length={1198} color="black" />
        <YAxis x={767} y={0} height={576} color="black" />
        {points1.map((point1, index) => (
          <Point key={`point1-${index}`} size={8} x={point1.easting} y={point1.northing} color="red" />
        ))}
        {points2.map((point2, index) => (
          <Point key={`point2-${index}`} size={8} x={point2.easting} y={point2.northing} color="green" />
        ))}
        {points3.map((point3, index) => (
          <Point key={`point3-${index}`} size={8} x={point3.easting} y={point3.northing} color="purple" />
        ))}
      </View>
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
      left: ((x * 10) + x + 769) - size / 2,
      top: (407 - (y * 10) - y) - size / 2,
      width: size,
      height: size,
      backgroundColor: color, // You can customize the color here
      borderRadius: size / 2,
    };
  
  return <View style={pointStyle} />;
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

export default CoordinateTest;