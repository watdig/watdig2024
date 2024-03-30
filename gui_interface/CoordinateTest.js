import React, { useState, useEffect } from 'react';
import { View, Text, StyleSheet } from 'react-native';
import Roslib from 'roslib';

const CoordinateTest = () => {
  const [points1, setPoints1] = useState([]);
  const [points2, setPoints2] = useState([]);
  const [points3, setPoints3] = useState([]);
  const [dynamicPoint, setDynamicPoint] = useState(null);

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
  useEffect(() => {

    const ros = new Roslib.Ros({
      url: 'ws://172.20.10.14:9090'
    });

    const environmentClient = new Roslib.Service({
      ros: ros,
      name: '/environment_csv_service',
      serviceType: 'interfacesarray/Environmentarray'
    });

    const checkpointsClient = new Roslib.Service({
      ros: ros,
      name: '/checkpoints_csv_service',
      serviceType: 'interfacesarray/Checkpointsarray'
    });

    const obstaclesClient = new Roslib.Service({
      ros: ros,
      name: '/obstacle_csv_service',
      serviceType: 'interfacesarray/Obstaclesarray'
    });

    // Define service request messages
    const environmentRequest = new Roslib.ServiceRequest({
      csv: 'environment' // Specify the CSV file name
    });

    const checkpointsRequest = new Roslib.ServiceRequest({
      csv: 'checkpoints' // Specify the CSV file name
    });

    const obstaclesRequest = new Roslib.ServiceRequest({
      csv: 'obstacles' // Specify the CSV file name
    });

    // Define callback functions to handle service responses
    const handleEnvironmentResponse = (response) => {
      console.log(response)
      setPoints3(response.array);
    };

    const handleCheckpointsResponse = (response) => {
      setPoints2(response.array);
    };

    const handleObstaclesResponse = (response) => {
      setPoints1(response.array);
    };



    // Call services to fetch data
    environmentClient.callService(environmentRequest, handleEnvironmentResponse);
    checkpointsClient.callService(checkpointsRequest, handleCheckpointsResponse);
    obstaclesClient.callService(obstaclesRequest, handleObstaclesResponse);

    return () => {
      ros.close();
    };
  }, []);


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


const Node = () => {
    return (
      <View style={styles.node} />
    );
  };

  

  const Point = ({ size, x, y, color }) => {
    // Only render the point if x and y coordinates are valid
    if (!isNaN(x) && !isNaN(y)) {
      const pointStyle = {
        position: 'absolute',
        left: ((x * 12) + 300) - size / 2,
        top: (492 - (y * 12)) - size / 2,
        width: size,
        height: size,
        backgroundColor: color, // You can customize the color here
        borderRadius: size / 2,
      };
  
      return <View style={pointStyle} />;
    } else {
      // If x or y coordinates are invalid (NaN), return null
      return null;
    }
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