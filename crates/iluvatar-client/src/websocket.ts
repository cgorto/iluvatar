export interface ServerMessage {
  type: 'Snapshot' | 'Update' | 'CameraStatus' | 'SystemStatus';
  data: unknown;
}

type MessageHandler = (msg: ServerMessage) => void;
type ConnectionHandler = (connected: boolean) => void;

export class WebSocketClient {
  private ws: WebSocket | null = null;
  private url: string;
  private messageHandlers: MessageHandler[] = [];
  private connectionHandlers: ConnectionHandler[] = [];
  private reconnectDelay = 1000;
  private maxReconnectDelay = 30000;
  private currentDelay = 1000;

  constructor(url: string) {
    this.url = url;
  }

  connect(): void {
    try {
      this.ws = new WebSocket(this.url);

      this.ws.onopen = () => {
        console.log('WebSocket connected');
        this.currentDelay = this.reconnectDelay;
        this.notifyConnectionChange(true);
        this.send({ type: 'Subscribe' });
      };

      this.ws.onmessage = (event) => {
        try {
          const msg = JSON.parse(event.data) as ServerMessage;
          this.notifyMessage(msg);
        } catch (e) {
          console.error('Failed to parse message:', e);
        }
      };

      this.ws.onclose = () => {
        console.log('WebSocket disconnected');
        this.notifyConnectionChange(false);
        this.scheduleReconnect();
      };

      this.ws.onerror = (error) => {
        console.error('WebSocket error:', error);
      };
    } catch (e) {
      console.error('Failed to create WebSocket:', e);
      this.scheduleReconnect();
    }
  }

  private scheduleReconnect(): void {
    console.log(`Reconnecting in ${this.currentDelay}ms...`);
    setTimeout(() => {
      this.connect();
    }, this.currentDelay);

    this.currentDelay = Math.min(this.currentDelay * 2, this.maxReconnectDelay);
  }

  send(data: unknown): void {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(data));
    }
  }

  onMessage(handler: MessageHandler): void {
    this.messageHandlers.push(handler);
  }

  onConnectionChange(handler: ConnectionHandler): void {
    this.connectionHandlers.push(handler);
  }

  private notifyMessage(msg: ServerMessage): void {
    for (const handler of this.messageHandlers) {
      handler(msg);
    }
  }

  private notifyConnectionChange(connected: boolean): void {
    for (const handler of this.connectionHandlers) {
      handler(connected);
    }
  }

  disconnect(): void {
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
  }
}
