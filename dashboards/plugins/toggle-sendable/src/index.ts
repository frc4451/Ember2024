import './garage-toggle-sendable';
import { addElements, addThemeRules } from '@frc-web-components/app';
import { toggleSendableDashboardConfig } from './garage-toggle-sendable';

addElements({
  'garage-toggle-sendable': toggleSendableDashboardConfig
}, 'Garage Plugins');

addThemeRules('dark', {
  '--garage-toggle-sendable-background': 'cadetblue',
  '--garage-toggle-sendable-color': 'black',
});

addThemeRules('light', {
  '--garage-toggle-sendable-background': 'cornflowerblue',
  '--garage-toggle-sendable-color': 'white',
});