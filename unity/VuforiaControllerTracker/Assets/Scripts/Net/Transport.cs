using System;
using System.Collections.Generic;
using System.IO;

#if UNITY_EDITOR

using System.Net;
using System.Net.Sockets;

#else

using System.Threading.Tasks;

#endif

using UnityEngine;

namespace Net
{
    public class MessageTruncatedException : Exception
    {
        public MessageTruncatedException()
        {
        }

        public MessageTruncatedException(string message)
          : base(message)
        {
        }

        public MessageTruncatedException(string message, Exception inner)
          : base(message, inner)
        {
        }
    }

    public class SessionRestartException : Exception
    {
        public SessionRestartException()
        {
        }

        public SessionRestartException(string message)
          : base(message)
        {
        }

        public SessionRestartException(string message, Exception inner)
          : base(message, inner)
        {
        }
    }

    public class ServerRestartException : Exception
    {
        public ServerRestartException()
        {
        }

        public ServerRestartException(string message)
          : base(message)
        {
        }

        public ServerRestartException(string message, Exception inner)
          : base(message, inner)
        {
        }
    }
}

#if UNITY_EDITOR

namespace Net
{
    //TODO: these somehow keep themselves alive
    public class Session
    {
        private TcpClient m_tcpClient;
        private NetworkStream m_stream;
        private readonly EndPoint m_remoteEndpoint;
        private readonly object m_lock = new object();
        private Action<Session, Exception> m_DisconnectHandler;
        private IMessageHandler m_messageHandler;
        private byte[] m_sizePrefix = new byte[4];
        private int m_sizePrefixBytesReceived = 0;
        private byte[] m_messageBuffer = null;
        private int m_messageBytesReceived = 0;

        internal Session(TcpClient tcpClient, Action<Session, Exception> DisconnectHandler, IMessageHandler messageHandler)
        {
            m_DisconnectHandler = DisconnectHandler != null ? DisconnectHandler : (Session session, Exception e) => { };
            m_messageHandler = messageHandler;
            m_tcpClient = tcpClient;
            m_tcpClient.NoDelay = true;
            m_tcpClient.SendBufferSize = 1024 * 1024;
            m_tcpClient.ReceiveBufferSize = 1024 * 1024;
            m_stream = m_tcpClient.GetStream();
            m_remoteEndpoint = m_tcpClient.Client.RemoteEndPoint;
        }

        ~Session()
        {
            Stop();
            m_stream = null;
        }

        public override string ToString()
        {
            return RemoteEndpoint.ToString();
        }

        private bool IsStopped()
        {
            return m_tcpClient == null;
        }

        public bool IsConnected
        {
            get
            {
                return m_tcpClient.Connected;
            }
        }

        public EndPoint RemoteEndpoint
        {
            get
            {
                return m_remoteEndpoint;
            }
        }

        internal void Start()
        {
            lock (m_lock)
            {
                if (IsStopped())
                {
                    throw new SessionRestartException("Session has been closed and cannot be restarted");
                }
            }

            m_sizePrefixBytesReceived = 0;
            BeginSizePrefix();
        }

        public void Stop()
        {
            lock (m_lock)
            {
                m_DisconnectHandler = null;
                if (IsStopped())
                {
                    return;
                }
                m_tcpClient.Close();
                m_tcpClient = null;
                m_stream.Close(); // not nulled out in case another thread calls Send() after session stopped
                m_messageHandler = null;
            }
        }

        public void Send(byte[] payloadBytes)
        {
            try
            {
                // Send total message size followed by payload
                UInt32 totalSize = 4 + (UInt32)payloadBytes.Length;
                byte[] sizePrefix = new byte[4];
                sizePrefix[0] = (byte)(totalSize & 0xff);
                sizePrefix[1] = (byte)((totalSize >> 8) & 0xff);
                sizePrefix[2] = (byte)((totalSize >> 16) & 0xff);
                sizePrefix[3] = (byte)((totalSize >> 24) & 0xff);
                m_stream.Write(sizePrefix, 0, sizePrefix.Length);
                m_stream.Write(payloadBytes, 0, payloadBytes.Length);
            }
            catch (ArgumentNullException e)
            {
                Debug.LogErrorFormat("Session: ArgumentNullException: {0}", e);
                throw;
            }
            catch (ObjectDisposedException e)
            {
                // This can occur if writes were enqueued after a disconnect and should be treated as non-fatal
                Debug.LogFormat("Session: ObjectDisposedException (most likely due to a write after Session stopped): {0}", e);
            }
            catch (IOException e)
            {
                lock (m_lock)
                {
                    Debug.LogErrorFormat("Session: IOException: {0}", e);
                    m_DisconnectHandler?.Invoke(this, e);
                    Stop();
                }
            }
            catch (SocketException e)
            {
                lock (m_lock)
                {
                    Debug.LogErrorFormat("Session: SocketException: {0}", e);
                    m_DisconnectHandler?.Invoke(this, e);
                    Stop();
                }
            }
            catch (Exception e)
            {
                lock (m_lock)
                {
                    Debug.LogErrorFormat("Session: Unknown exception: {0}", e);
                    m_DisconnectHandler?.Invoke(this, e);
                    Stop();
                }
            }
        }

        private void AllocateNewMessageBuffer()
        {
            // Message buffer holds the entire message, including the size prefix
            m_messageBuffer = new byte[Utils.GetMessageSize(m_sizePrefix)];
            Buffer.BlockCopy(m_sizePrefix, 0, m_messageBuffer, 0, m_sizePrefix.Length);
            m_messageBytesReceived = m_sizePrefix.Length;
        }

        private void BeginSizePrefix()
        {
            int bytesRemaining = m_sizePrefix.Length - m_sizePrefixBytesReceived;
            m_stream.BeginRead(m_sizePrefix, m_sizePrefixBytesReceived, bytesRemaining, new AsyncCallback(FinishSizePrefix), m_stream);
        }

        private void FinishSizePrefix(IAsyncResult result)
        {
            int numBytesReceived = 0;

            try
            {
                numBytesReceived = m_stream.EndRead(result);
            }
            catch (Exception e)
            {
                lock (m_lock)
                {
                    m_DisconnectHandler?.Invoke(this, e);
                    Stop();
                }
                return;
            }

            if (numBytesReceived == 0)
            {
                lock (m_lock)
                {
                    string errorMessage = string.Format("Connection closed (read {0} of {1} bytes expected)", m_sizePrefixBytesReceived, m_sizePrefix.Length);
                    m_DisconnectHandler?.Invoke(this, new MessageTruncatedException(errorMessage));
                    IsStopped();
                }
                return;
            }

            m_sizePrefixBytesReceived += numBytesReceived;

            if (m_sizePrefixBytesReceived == m_sizePrefix.Length)
            {
                // Size of message obtained; read payload next
                AllocateNewMessageBuffer();
                BeginPayload();
            }
            else
            {
                // Still more to receive...
                BeginSizePrefix();
            }
        }

        private void BeginPayload()
        {
            int bytesRemaining = m_messageBuffer.Length - m_messageBytesReceived;
            m_stream.BeginRead(m_messageBuffer, m_messageBytesReceived, bytesRemaining, new AsyncCallback(FinishPayload), m_stream);
        }

        private void FinishPayload(IAsyncResult result)
        {
            int numBytesReceived = 0;

            try
            {
                numBytesReceived = m_stream.EndRead(result);
            }
            catch (Exception e)
            {
                lock (m_lock)
                {
                    m_DisconnectHandler?.Invoke(this, e);
                    Stop();
                }
                return;
            }

            UInt32 totalMessageSize = Utils.GetMessageSize(m_sizePrefix);
            bool hasBody = totalMessageSize > m_sizePrefix.Length;

            if (numBytesReceived == 0 && hasBody)
            {
                lock (m_lock)
                {
                    string errorMessage = string.Format("Connection closed (read {0} of {1} bytes expected)", m_messageBytesReceived, m_messageBuffer.Length);
                    m_DisconnectHandler?.Invoke(this, new MessageTruncatedException(errorMessage));
                }
                return;
            }

            m_messageBytesReceived += numBytesReceived;

            if (m_messageBytesReceived == m_messageBuffer.Length)
            {
                // Obtained complete message
                if (m_messageHandler != null)
                {
                    m_messageHandler.HandleMessage(this, m_messageBuffer);
                }

                // Read next message
                m_messageBuffer = null;
                m_sizePrefixBytesReceived = 0;
                BeginSizePrefix();
            }
            else
            {
                // Still more to receive...
                BeginPayload();
            }
        }
    }

    public class TCPClient
    {
        public void Connect(string hostname, int port, Action<Session, Exception> ConnectHandler, Action<Session, Exception> DisconnectHandler, IMessageHandler messageHandler)
        {
            bool failed = true;
            Exception exception = null;
            Session session = null;

            try
            {
                TcpClient client = new TcpClient(hostname, port);
                session = new Session(client, DisconnectHandler, messageHandler);
                session.Start();
                failed = false;
            }
            catch (ArgumentNullException e)
            {
                exception = e;
            }
            catch (SocketException e)
            {
                // Connections that are refused trigger this exception
                exception = e;
            }

            if (failed)
            {
                // We indicate connect failures by calling the ConnectHandler with a null session
                ConnectHandler(null, exception);
            }
            else
            {
                // Connection must have succeeded
                ConnectHandler(session, exception);
            }
        }
    }

    public class TCPServer
    {
        private TcpListener m_tcpListener = null;
        private object m_lock = new object();
        private HashSet<Session> m_pendingSessions = new HashSet<Session>();

        private class AcceptData
        {
            public TcpListener tcpListener;
            public int port;
            public Action<Session> AcceptHandler;
            public Action<Session, Exception> DisconnectHandler;
            public IMessageHandler messageHandler;

            public AcceptData(TcpListener tcpListener, int port, Action<Session> AcceptHandler, Action<Session, Exception> DisconnectHandler, IMessageHandler messageHandler)
            {
                this.tcpListener = tcpListener;
                this.port = port;
                this.AcceptHandler = AcceptHandler;
                this.DisconnectHandler = DisconnectHandler;
                this.messageHandler = messageHandler;
            }
        }

        ~TCPServer()
        {
            Stop();
        }

        public void Start(int port, Action<Session> AcceptHandler, Action<Session, Exception> DisconnectHandler, IMessageHandler messageHandler)
        {
            if (m_tcpListener != null)
            {
                throw new ServerRestartException("Server already started");
            }
            StartAccepting(port, AcceptHandler, DisconnectHandler, messageHandler);
        }

        public void Stop()
        {
            if (m_tcpListener != null)
            {
                m_tcpListener.Stop();
                m_tcpListener = null;
            }
        }

        // Listens on all interfaces
        private void StartAccepting(int port, Action<Session> AcceptHandler, Action<Session, Exception> DisconnectHandler, IMessageHandler messageHandler)
        {
            if (m_tcpListener == null)
            {
                m_tcpListener = new TcpListener(IPAddress.Any, port);
                m_tcpListener.Server.NoDelay = true;
                m_tcpListener.Start();
            }

            AcceptData data = new AcceptData(m_tcpListener, port, AcceptHandler, DisconnectHandler, messageHandler);
            m_tcpListener.BeginAcceptTcpClient(new AsyncCallback(FinishAccept), data);
        }

        // Listen on specific interface
        private void StartAccepting(string interfaceIP, int port, Action<Session> AcceptHandler, Action<Session, Exception> DisconnectHandler, IMessageHandler messageHandler)
        {
            if (m_tcpListener == null)
            {
                IPEndPoint ourEndPoint = new IPEndPoint(IPAddress.Parse(interfaceIP), port);
                m_tcpListener = new TcpListener(ourEndPoint);
                m_tcpListener.Server.NoDelay = true;
                m_tcpListener.Start();
            }

            AcceptData data = new AcceptData(m_tcpListener, port, AcceptHandler, DisconnectHandler, messageHandler);
            m_tcpListener.BeginAcceptTcpClient(new AsyncCallback(FinishAccept), data);
        }

        private void FinishAccept(IAsyncResult result)
        {
            AcceptData data = (AcceptData)result.AsyncState;
            TcpClient tcpClient = null;

            try
            {
                tcpClient = data.tcpListener.EndAcceptTcpClient(result);
            }
            catch (ObjectDisposedException)
            {
                // Stop() was called
                return;
            }

            Debug.LogFormat("TCPServer: Accepted connection from {0}", tcpClient.Client.RemoteEndPoint);

            void DisconnectIntercepter(Session session, Exception e)
            {
                // Forward to user and remove from pending set
                data.DisconnectHandler(session, e);
                lock (m_lock)
                {
                    m_pendingSessions.Remove(session);
                }
            }

            // Create and start a new session
            Session session = new Session(tcpClient, DisconnectIntercepter, data.messageHandler);
            lock (m_lock)
            {
                m_pendingSessions.Add(session);
            }
            session.Start();
            data.AcceptHandler?.Invoke(session);

            // Continue listening for more connections
            m_tcpListener.BeginAcceptTcpClient(new AsyncCallback(FinishAccept), data);
        }
    }
}

#else

namespace Net
{
  public class Session
  {
    private readonly object m_lock = new object();  
    private Windows.Networking.Sockets.StreamSocket m_socket;
    private Windows.Storage.Streams.DataWriter m_writer;
    private System.IO.Stream m_inputStream;
    private Action<Session, Exception> m_DisconnectHandler;
    private IMessageHandler m_messageHandler;
    private System.Threading.CancellationTokenSource m_cancellationSource = new System.Threading.CancellationTokenSource();
    private Task m_readMessagesTask = null;

    public Session(Windows.Networking.Sockets.StreamSocket socket, Action<Session, Exception> DisconnectHandler, IMessageHandler messageHandler)
    {
      m_DisconnectHandler = DisconnectHandler;
      m_socket = socket;
      m_writer = new Windows.Storage.Streams.DataWriter(m_socket.OutputStream);
      m_inputStream = m_socket.InputStream.AsStreamForRead();
      m_messageHandler = messageHandler;
    }

    ~Session()
    {
      Stop();
      m_readMessagesTask.Wait();
      m_writer = null;
      m_inputStream = null;
    }

    public void Start()
    {
      m_readMessagesTask = ReadMessages();
    }

    public void Stop()
    {
      lock (m_lock)
      {
        m_DisconnectHandler = null;
        if (IsStopped)
        {
          return;
        }
        IsStopped = true;
        m_cancellationSource.Cancel();
        m_inputStream.Dispose();
        m_messageHandler = null;
        //TODO: should we dispose m_writer?
      }
    }

    public string RemoteEndpoint
    {
      get
      {
        return m_socket.Information.RemoteAddress.ToString() + ":" + m_socket.Information.RemotePort.ToString();
      }
    }

    public override string ToString()
    {
      return RemoteEndpoint;
    }

    private Task ReadMessages()
    {
      //TODO: handle disconnections and exceptions? A return value of 0 from ReadAsync indicates an error

      return Task.Run(
        async () =>
        {
          byte[] sizePrefixBuffer = new byte[4];
          byte[] messageBuffer = null;

          try
          {
            while (true)
            {
              // Read message size prefix
              Int32 bytesRead = 0;
              while (bytesRead < sizePrefixBuffer.Length)
              {
                Task<Int32> sizePrefixTask = m_inputStream.ReadAsync(sizePrefixBuffer, bytesRead, sizePrefixBuffer.Length - bytesRead, m_cancellationSource.Token);
                await sizePrefixTask;
                bytesRead += sizePrefixTask.Result;
                if (m_cancellationSource.IsCancellationRequested)
                {
                  return;
                }
              }

              // Read remainder of message
              UInt32 size = Utils.GetMessageSize(sizePrefixBuffer);
              messageBuffer = new byte[size];
              Buffer.BlockCopy(sizePrefixBuffer, 0, messageBuffer, 0, sizePrefixBuffer.Length);
              bytesRead = 0;
              while (bytesRead < messageBuffer.Length - sizePrefixBuffer.Length)
              {
                Task<Int32> bodyTask = m_inputStream.ReadAsync(messageBuffer, sizePrefixBuffer.Length + bytesRead, (int)size - sizePrefixBuffer.Length - bytesRead, m_cancellationSource.Token);
                await bodyTask;
                bytesRead += bodyTask.Result;
                if (m_cancellationSource.IsCancellationRequested)
                {
                  return;
                }
              }

              // Handle message
              if (m_messageHandler != null)
              {
                m_messageHandler.HandleMessage(this, messageBuffer);
              }
            }
          }
          catch (Exception exception)
          {
            m_DisconnectHandler?.Invoke(this, exception);
            Stop();
          }
        });
    }

    public async Task Send(byte[] payloadBytes)
    {
      try
      {
        // Send total message size followed by payload
        UInt32 totalSize = 4 + (UInt32)payloadBytes.Length;
        byte[] sizePrefix = new byte[4];
        sizePrefix[0] = (byte)(totalSize & 0xff);
        sizePrefix[1] = (byte)((totalSize >> 8) & 0xff);
        sizePrefix[2] = (byte)((totalSize >> 16) & 0xff);
        sizePrefix[3] = (byte)((totalSize >> 24) & 0xff);
        m_writer.WriteBytes(sizePrefix);
        m_writer.WriteBytes(payloadBytes);
        await m_writer.StoreAsync();
      }
      catch (Exception exception)
      {
        // Unknown status means error is fatal and retry will likely fail
        if (Windows.Networking.Sockets.SocketError.GetStatus(exception.HResult) == Windows.Networking.Sockets.SocketErrorStatus.Unknown)
        {
        }
        m_DisconnectHandler?.Invoke(this, exception);
        Stop();
      }
    }

    private bool IsStopped
    {
      get;
      set;
    }
  }

  public class TCPClient
  {
    public async void Connect(string hostname, int port, Action<Session, Exception> ConnectHandler, Action<Session, Exception> DisconnectHandler, IMessageHandler messageHandler)
    {
      bool blockUntilConnected = true;

      Task connectionTask = Task.Run(
        async () =>
        {
          Session session = null;
          Exception exception = null;

          try
          {
            Windows.Networking.Sockets.StreamSocket socket = new Windows.Networking.Sockets.StreamSocket();
            socket.Control.NoDelay = true;

            Windows.Networking.HostName host = new Windows.Networking.HostName(hostname);
            await socket.ConnectAsync(host, port.ToString());

            session = new Session(socket, DisconnectHandler, messageHandler);
            session.Start();
          }
          catch (Exception e)
          {
            exception = e;
          }

          if (exception != null)
          {
            // Connection failures indicated by calling ConnectHandler with null session
            ConnectHandler?.Invoke(null, exception);
          }
          else
          {
            // Successful connection
            ConnectHandler?.Invoke(session, null);
          }
        }
      );

      if (blockUntilConnected)
      {
        connectionTask.Wait();
      }

      await connectionTask;
    }
  }

  public class TCPServer
  {
    private Windows.Networking.Sockets.StreamSocketListener m_listener = null;
    private Action<Session> m_AcceptHandler = null;
    private Action<Session, Exception> m_DisconnectHandler = null;
    private IMessageHandler m_messageHandler = null;

    ~TCPServer()
    {
      StopListening();
    }

    public async void Start(int port, Action<Session> AcceptHandler, Action<Session, Exception> DisconnectHandler, IMessageHandler messageHandler)
    {
      if (m_listener != null)
      {
        throw new Exception("Server already started");
      }
      m_AcceptHandler = AcceptHandler;
      m_DisconnectHandler = DisconnectHandler;
      m_messageHandler = messageHandler;
      await StartAccepting(port);
    }

    public void Stop()
    {
      StopListening();
    }

    private void OnConnectionReceived(Windows.Networking.Sockets.StreamSocketListener listener, Windows.Networking.Sockets.StreamSocketListenerConnectionReceivedEventArgs args)
    {
      Session session = new Session(args.Socket, m_DisconnectHandler, m_messageHandler);
      session.Start();
      m_AcceptHandler(session);
    }

    private async Task StartAccepting(int port)
    {
      try
      {
        m_listener = new Windows.Networking.Sockets.StreamSocketListener();
        m_listener.Control.NoDelay = true;  // disable Nagle algorithm
                                            //m_listener.Control.KeepAlive = true;
        m_listener.ConnectionReceived += OnConnectionReceived;
        //Windows.Networking.HostName hostname = new Windows.Networking.HostName("localhost");
        await m_listener.BindEndpointAsync(null, port.ToString());  // specifying localhost doesn't work
        Debug.LogFormat("Listening on port {0}", m_listener.Information.LocalPort.ToString());
      }
      catch (UnauthorizedAccessException exception)
      {
        Debug.LogFormat("Unable to accept connections due to lack of privileges: {0}", exception.Message);
        throw;
      }
    }

    private void StopListening()
    {
      if (m_listener != null)
      {
        m_listener.ConnectionReceived -= OnConnectionReceived;
        m_listener.Dispose();
        m_listener = null;
      }
    }
  }
}

#endif
