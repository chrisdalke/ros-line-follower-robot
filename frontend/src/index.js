import React from 'react';
import ReactDOM from 'react-dom';
import './assets/css/index.scss';
import MainPage from './components/pages/MainPage/MainPage';
import { FocusStyleManager } from "@blueprintjs/core";

FocusStyleManager.onlyShowFocusOnTabs();
ReactDOM.render(
  <React.StrictMode>
    <MainPage />
  </React.StrictMode>,
  document.getElementById('root')
);
