import React, { useState, useEffect } from 'react';
import { View, Text, StyleSheet } from 'react-native';
import Roslib from 'roslib';


const CoordinateTest = () => {
  const [points, setPoints] = useState([]);
  let pointCounter = 0;
  const renderGrid = () => {
    const grid = [];
    for (let row = 0; row < 48; row++) {
        const rowNodes = [];
        for (let col = 0; col < 100; col++) {
          if (row === 24) { // Check if it's the 15th row
            rowNodes.push(<Node key={`node-${row}-${col}`} style={styles.boldNode} />);
          } else {
            rowNodes.push(<Node key={`node-${row}-${col}`} />);
          }
        }
      grid.push(<View key={`row-${row}`} style={styles.row}>{rowNodes}</View>);
    }
    return grid;
  };
  useEffect(() => {
    const ros = new Roslib.Ros({
      url: 'ws://192.168.246.130:9090'
    });

    const topicListener = new Roslib.Topic({
      ros: ros,
      name: '/obstacle_csv_topic',
      messageType: 'interfaces/Obstacles'
    });

    const subscribeToRosTopic = (message) => {
      // Parse the message
      const newPoint = {
        key: pointCounter++,
        easting: message.easting,
        northing: message.northing,
      };
      setPoints(prevPoints => [...prevPoints, newPoint]);
    };

    topicListener.subscribe(subscribeToRosTopic);

    return () => {
      topicListener.unsubscribe();
      ros.close();
    };
  }, []);




  return (
    <View style={styles.container}>
      <View style={styles.gridContainer}>
        {renderGrid()}
        {points.map((point, index) => (
          <Point key={`${point.easting}-${point.northing}-${index}`} size={8} x={point.easting} y={point.northing} />
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

  const Point = ({ size, x, y }) => {
    const pointStyle = {
      position: 'absolute',
      left: ((x * 10) + x + 768) - size / 2,
      top: (288 - (y * 10) - y) - size / 2,
      width: size,
      height: size,
      backgroundColor: 'red', // You can customize the color here
      borderRadius: size / 2,
    };
  
  return <View style={pointStyle} />;
};


const styles = StyleSheet.create({
  container: {
    flex: 1,
    flexDirection: 'row',
  },
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