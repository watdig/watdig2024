import React, { useState, useEffect } from 'react';
import { View, StyleSheet } from 'react-native';
import Roslib from 'roslib';

const CoordinateTest = () => {
  const [data, setData] = useState({
    name: '',
    easting: 0,
    northing: 0,
    elevation: 0,
    bounding_radius: 0
  });
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
      const parsedData = {
        name: message.name,
        easting: message.easting,
        northing: message.northing,
        elevation: message.elevation,
        bounding_radius: message.bounding_radius
      };
      setData(parsedData);
    };

    topicListener.subscribe(subscribeToRosTopic);

    return () => {
      topicListener.unsubscribe();
      ros.close();
    };
  }, []);

  return (
    <View style={styles.gridContainer}>
      {renderGrid()}
      <Point size={data.bounding_radius} x={data.easting} y={data.northing} />
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

export default CoordinateTest;