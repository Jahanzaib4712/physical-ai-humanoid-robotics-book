id: voice-to-action
title: Voice-to-Action with OpenAI Whisper
sidebar_position: 2
---

# Chapter 2: Voice-to-Action with OpenAI Whisper

## Natural Robot Interaction

**Vision:** Robots that respond to spoken commands, just like digital assistants (Alexa, Siri) but with physical actions.
```
Human: "Please bring me a glass of water"
         ↓
      [Whisper]
         ↓
     "bring me a glass of water" (text)
         ↓
      [VLA Model]
         ↓
   [Navigate, Grasp, Carry, Deliver]

Why Voice Commands?
Advantages
✅ Natural: Most intuitive human interface
✅ Hands-free: User doesn't need controller/keyboard
✅ Flexible: Can specify complex tasks in natural language
✅ Accessible: Works for users with limited mobility
Challenges
❌ Noisy environments: Background sounds interfere
❌ Accents/dialects: Model must handle variations
❌ Ambiguity: "Get the cup" - which cup?
❌ Latency: Real-time transcription needed

OpenAI Whisper
What is Whisper?
State-of-the-art automatic speech recognition (ASR) model:

680,000 hours of multilingual training data
99 languages supported
Robust to accents, noise, technical terms

Model Sizes
ModelParametersSpeedAccuracytiny39MVery fastGoodbase74MFastBettersmall244MMediumVery goodmedium769MSlowExcellentlarge1550MVery slowBest
For robotics: Use base or small for real-time performance.

Installation and Setup
Install Whisper
bashpip install openai-whisper
# Or for faster inference:
pip install faster-whisper
Basic Usage
pythonimport whisper

# Load model
model = whisper.load_model("base")

# Transcribe audio file
result = model.transcribe("command.wav")
print(result["text"])
# Output: "Pick up the red block"

Real-Time Voice Commands
Microphone Streaming
pythonimport whisper
import pyaudio
import wave
import numpy as np

class VoiceCommandListener:
    def __init__(self, model_size="base"):
        self.model = whisper.load_model(model_size)
        self.audio = pyaudio.PyAudio()
        
        # Audio parameters
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000  # Whisper expects 16kHz
        self.chunk = 1024
        self.record_seconds = 5  # Record 5 seconds at a time
        
    def listen(self):
        """Record audio and transcribe"""
        stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )
        
        print("Listening...")
        frames = []
        
        for _ in range(0, int(self.rate / self.chunk * self.record_seconds)):
            data = stream.read(self.chunk)
            frames.append(data)
        
        stream.stop_stream()
        stream.close()
        
        # Convert to numpy array
        audio_data = np.frombuffer(b''.join(frames), dtype=np.int16)
        audio_data = audio_data.astype(np.float32) / 32768.0  # Normalize
        
        # Transcribe
        result = self.model.transcribe(audio_data, language="en")
        command = result["text"].strip()
        
        print(f"Heard: {command}")
        return command
    
    def close(self):
        self.audio.terminate()

# Usage
listener = VoiceCommandListener()
command = listener.listen()

Integrating with ROS 2
Voice Command Node
pythonimport rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import pyaudio
import numpy as np

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')
        
        # Publisher for transcribed commands
        self.command_pub = self.create_publisher(String, 'voice_command', 10)
        
        # Load Whisper model
        self.model = whisper.load_model("base")
        self.get_logger().info("Whisper model loaded")
        
        # Audio setup
        self.audio = pyaudio.PyAudio()
        self.rate = 16000
        self.chunk = 1024
        
        # Start listening loop
        self.timer = self.create_timer(0.1, self.listen_callback)
        self.is_listening = False
        
    def listen_callback(self):
        """Continuously listen for voice commands"""
        if not self.is_listening:
            self.is_listening = True
            command = self.capture_and_transcribe()
            
            if command:
                msg = String()
                msg.data = command
                self.command_pub.publish(msg)
                self.get_logger().info(f'Published command: {command}')
            
            self.is_listening = False
    
    def capture_and_transcribe(self):
        """Record 3 seconds and transcribe"""
        stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )
        
        frames = []
        for _ in range(0, int(self.rate / self.chunk * 3)):
            data = stream.read(self.chunk, exception_on_overflow=False)
            frames.append(data)
        
        stream.stop_stream()
        stream.close()
        
        # Convert and transcribe
        audio_data = np.frombuffer(b''.join(frames), dtype=np.int16).astype(np.float32) / 32768.0
        
        result = self.model.transcribe(audio_data, language="en", fp16=False)
        return result["text"].strip()
    
    def destroy_node(self):
        self.audio.terminate()
        super().destroy_node()

def main():
    rclpy.init()
    node = VoiceCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

Voice Activation (Wake Word)
Using Porcupine
bashpip install pvporcupine
pythonimport pvporcupine
import pyaudio

class WakeWordListener:
    def __init__(self, access_key, keywords=["jarvis"]):
        self.porcupine = pvporcupine.create(
            access_key=access_key,
            keywords=keywords
        )
        
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            rate=self.porcupine.sample_rate,
            channels=1,
            format=pyaudio.paInt16,
            input=True,
            frames_per_buffer=self.porcupine.frame_length
        )
    
    def listen_for_wake_word(self):
        """Block until wake word detected"""
        print(f"Listening for wake word...")
        
        while True:
            pcm = self.stream.read(self.porcupine.frame_length)
            pcm = np.frombuffer(pcm, dtype=np.int16)
            
            keyword_index = self.porcupine.process(pcm)
            if keyword_index >= 0:
                print(f"Wake word detected!")
                return True
    
    def close(self):
        self.stream.close()
        self.audio.terminate()
        self.porcupine.delete()

# Usage
wake_listener = WakeWordListener(access_key="YOUR_KEY", keywords=["jarvis"])
if wake_listener.listen_for_wake_word():
    command = voice_listener.listen()  # Now listen for actual command

Complete Voice-to-Action System
Architecture
Microphone → Wake Word → Whisper → VLA Model → Robot
              (Porcupine)            (OpenVLA)
Implementation
pythonimport rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import whisper
import pyaudio
import numpy as np

class VoiceControlledRobot(Node):
    def __init__(self):
        super().__init__('voice_controlled_robot')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Whisper model
        self.model = whisper.load_model("base")
        
        # Command mapping
        self.command_map = {
            "forward": (0.5, 0.0),
            "backward": (-0.5, 0.0),
            "left": (0.0, 0.5),
            "right": (0.0, -0.5),
            "stop": (0.0, 0.0)
        }
        
        self.get_logger().info("Voice-controlled robot ready")
    
    def listen_and_execute(self):
        """Main loop: listen → parse → execute"""
        command_text = self.capture_voice()
        
        if command_text:
            self.get_logger().info(f"Command: {command_text}")
            self.execute_command(command_text)
    
    def capture_voice(self):
        """Capture and transcribe voice"""
        # (Audio capture code from above)
        audio_data = self.record_audio(duration=3)
        result = self.model.transcribe(audio_data, language="en")
        return result["text"].strip().lower()
    
    def execute_command(self, command_text):
        """Map text to robot action"""
        cmd = Twist()
        
        for keyword, (linear, angular) in self.command_map.items():
            if keyword in command_text:
                cmd.linear.x = linear
                cmd.angular.z = angular
                self.cmd_vel_pub.publish(cmd)
                self.get_logger().info(f"Executing: {keyword}")
                return
        
        # If no match, try VLA model for complex commands
        self.get_logger().info("No simple command match, using VLA...")
        self.execute_vla_command(command_text)
    
    def execute_vla_command(self, instruction):
        """Use VLA for complex manipulation tasks"""
        # Get camera image
        image = self.get_camera_image()
        
        # VLA inference
        action = self.vla_model(image, instruction)
        
        # Execute action
        self.robot.execute(action)

def main():
    rclpy.init()
    robot = VoiceControlledRobot()
    
    try:
        while rclpy.ok():
            robot.listen_and_execute()
    except KeyboardInterrupt:
        pass
    
    robot.destroy_node()
    rclpy.shutdown()

Handling Ambiguity
Clarification Dialogues
pythondef resolve_ambiguity(self, command):
    """Ask for clarification if command ambiguous"""
    if "cup" in command and not self.has_specific_color(command):
        self.speak("Which cup? Please say the color.")
        clarification = self.capture_voice()
        command = f"{clarification} {command}"
    
    return command

def has_specific_color(self, text):
    colors = ["red", "blue", "green", "yellow", "black", "white"]
    return any(color in text for color in colors)

Text-to-Speech Feedback
Using gTTS (Google Text-to-Speech)
bashpip install gtts pygame
pythonfrom gtts import gTTS
import pygame
import os

class RobotSpeaker:
    def __init__(self):
        pygame.mixer.init()
    
    def speak(self, text):
        """Convert text to speech and play"""
        tts = gTTS(text=text, lang='en')
        tts.save("response.mp3")
        
        pygame.mixer.music.load("response.mp3")
        pygame.mixer.music.play()
        
        while pygame.mixer.music.get_busy():
            pygame.time.Clock().tick(10)
        
        os.remove("response.mp3")

# Usage
speaker = RobotSpeaker()
speaker.speak("I am picking up the red block")

Optimization for Real-Time
Faster Whisper
bashpip install faster-whisper
pythonfrom faster_whisper import WhisperModel

# Use CPU INT8 or GPU FP16
model = WhisperModel("base", device="cuda", compute_type="float16")

# Transcribe (much faster)
segments, info = model.transcribe("audio.wav")
for segment in segments:
    print(f"{segment.text}")
Speed improvement: 2-4x faster than standard Whisper

Practice Exercises
1. Basic Setup:

Install Whisper
Record 5-second audio file
Transcribe and print result

2. Voice-Controlled Navigation:

Create ROS 2 node
Listen for: "forward", "back", "left", "right", "stop"
Publish to /cmd_vel
Test in simulation

3. VLA Integration:

Combine Whisper with OpenVLA
Voice command → VLA action
Test on manipulation task

4. Complete System:

Wake word detection
Voice command capture
VLA inference
TTS feedback

Next → Chapter 3: LLM Cognitive Planning