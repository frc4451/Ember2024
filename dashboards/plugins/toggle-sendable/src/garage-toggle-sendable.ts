import { WebbitConfig } from '@webbitjs/webbit';
import { html, css, LitElement, TemplateResult } from 'lit';
import { property } from 'lit/decorators.js';

export const toggleSendableDashboardConfig: Partial<WebbitConfig> = {
  dashboard: {
    displayName: 'Toggle Sendable',
  },
  properties: {
    options: {
      type: 'Array',
      changeEvent: 'optionsUpdate',
      defaultValue: ['On', 'Off'],
      input: { type: 'StringArray' },
    },
    selected: {
      primary: true,
      type: 'String',
      changeEvent: 'change',
      input: {
        type: 'StringDropdown',
        allowCustomValues: false,
        getOptions({ options }: { options: string[] }): string[] {
          return options;
        },
      },
    },
    active: {
      type: 'String'
    },
    direction: {
      type: 'String',
      defaultValue: 'vertical',
      input: {
        type: 'StringDropdown',
        allowCustomValues: false,
        getOptions(): string[] {
          return ['vertical', 'horizontal'];
        },
      },
    },
  },
};

/**
 * Works the same as a Toggle Group, but can interact with the same functionality
 * as a `SendableChooser` from the WPILib API.
 */
export class ToggleSendable extends LitElement {
  @property({ type: Array }) options = ['On', 'Off'];
  @property({ type: String }) selected = '';
  @property({ type: String }) active = '';
  @property({ type: String }) direction = 'vertical';

  static styles = css`
    :host {
      display: inline-flex;
      flex-direction: column;
      width: 150px;
      height: 300px;
      gap: 0;
      font-family: sans-serif;
      font-size: 16px;
      letter-spacing: 0.5px;
    }

    button {
      width: 100%;
      height: 100%;
      margin: 0;
      border: none;
      font-size: inherit;
      font-family: inherit;
      letter-spacing: inherit;
      background: var(--frc-button-background-color, rgb(230, 230, 230));
      color: var(--frc-button-text-color, black);
      flex: 1;
    }

    .toggled {
      background: var(--frc-button-toggled-background-color, black);
      color: var(--frc-button-toggled-text-color, white);
      font-weight: bold;
    }

    [part='button'] {
      border-radius: 0;
      margin: 0;
      flex: 1;
      font-size: inherit;
      height: 100%;
      padding: 0;
    }
  `;

  async setSelected(value: string): Promise<void> {
    this.selected = value;
  }

  updated(changedProps: Map<string, unknown>): void {
    if (changedProps.has('options') && !this.options.includes(this.selected)) {
      this.selected = this.options[0] ?? '';
    }

    // dispatch events
    if (changedProps.has('selected')) {
      this.#dispatchChange();
    }

    if (changedProps.has('options')) {
      this.#dispatchOptionsUpdate();
    }

    if (changedProps.has('direction')) {
      this.style.setProperty(
        'flex-direction',
        this.direction === 'vertical' ? 'column' : 'row'
      );
    }
  }

  #dispatchChange(): void {
    this.dispatchEvent(
      new CustomEvent('change', {
        detail: { selected: this.selected },
        bubbles: true,
        composed: true,
      })
    );
  }

  #dispatchOptionsUpdate(): void {
    this.dispatchEvent(
      new CustomEvent('optionsUpdate', {
        detail: { options: this.options },
        bubbles: true,
        composed: true,
      })
    );
  }

  render(): TemplateResult {
    return html`
      ${this.options.map(
        (option) => html`
          <button
            class="${this.active === option ? 'toggled' : ''}"
            @click="${() => this.setSelected(option)}"
          >
            ${option}
          </button>
        `
      )}
    `;
  }
}

export default ToggleSendable;

if (!customElements.get('garage-toggle-sendable')) {
  customElements.define('garage-toggle-sendable', ToggleSendable);
}

declare global {
  interface HTMLElementTagNameMap {
    'garage-toggle-sendable': ToggleSendable;
  }
}