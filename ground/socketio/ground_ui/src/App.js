import React, { Component } from 'react';
import logo from './logo.svg';
import './App.css';

// Import from node_modules. This is just a fancier version of 'require()'
import socketIOClient from 'socket.io-client';

class App extends Component {
  // initialize class member variables
  socketHost = window.location.hostname; // read from browser URL
  socketPort = 8081;
  socket = null;

  // initialize React state
  state = {
    telemetry: null
  };

  constructor() {
    super();
    // initiate socket connection to http://localhost:8081 on the "/ui" namespace
    this.socket = socketIOClient(this.socketHost+':'+this.socketPort+'/ui', { transports: ['websocket'] });
    this.socket.on('telemetry', (data) => {
      console.log("Received Telemetry: " + data);
      // Update telemetry in the React state, thereby causing a re-render
      this.setState({ telemetry: data });
    });
  }

  render() {
    return (
      <div className="App">
        {/* Display current telemetry */}
        <div>The Telemetry is: {this.state.telemetry}</div>
      </div>
    );
  }
}

export default App;
