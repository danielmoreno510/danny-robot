// Danny - Codifica en MP4 (H.264) fotogramas RGB crudos con AVFoundation de macOS, sin ffmpeg.
// Uso: swift analysis/encode_video.swift <fotogramas.rgb> <ancho> <alto> <fps> <salida.mp4>
// El archivo de entrada lleva los fotogramas seguidos, ancho x alto x 3 bytes cada uno, filas de arriba abajo.
import AVFoundation
import Foundation

let args = CommandLine.arguments
let input = URL(fileURLWithPath: args[1])
let width = Int(args[2])!, height = Int(args[3])!, fps = Int32(args[4])!
let output = URL(fileURLWithPath: args[5])
try? FileManager.default.removeItem(at: output)

let raw = try Data(contentsOf: input)
let frameBytes = width * height * 3
let count = raw.count / frameBytes

let writer = try AVAssetWriter(outputURL: output, fileType: .mp4)
let settings: [String: Any] = [AVVideoCodecKey: AVVideoCodecType.h264, AVVideoWidthKey: width, AVVideoHeightKey: height]
let input_ = AVAssetWriterInput(mediaType: .video, outputSettings: settings)
let adaptor = AVAssetWriterInputPixelBufferAdaptor(assetWriterInput: input_, sourcePixelBufferAttributes: [
  kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_32BGRA, kCVPixelBufferWidthKey as String: width,
  kCVPixelBufferHeightKey as String: height,
])
writer.add(input_)
writer.startWriting()
writer.startSession(atSourceTime: .zero)

raw.withUnsafeBytes { (bytes: UnsafeRawBufferPointer) in
  for frame in 0..<count {
    while !input_.isReadyForMoreMediaData { usleep(1000) }
    var buffer: CVPixelBuffer?
    CVPixelBufferPoolCreatePixelBuffer(nil, adaptor.pixelBufferPool!, &buffer)
    CVPixelBufferLockBaseAddress(buffer!, [])
    let base = CVPixelBufferGetBaseAddress(buffer!)!.assumingMemoryBound(to: UInt8.self)
    let stride = CVPixelBufferGetBytesPerRow(buffer!)
    for y in 0..<height {
      for x in 0..<width {
        let s = frame * frameBytes + (y * width + x) * 3
        let d = y * stride + x * 4
        base[d] = bytes[s + 2]; base[d + 1] = bytes[s + 1]; base[d + 2] = bytes[s]; base[d + 3] = 255
      }
    }
    CVPixelBufferUnlockBaseAddress(buffer!, [])
    adaptor.append(buffer!, withPresentationTime: CMTime(value: CMTimeValue(frame), timescale: fps))
  }
}
input_.markAsFinished()
let done = DispatchSemaphore(value: 0)
writer.finishWriting { done.signal() }
done.wait()
print("Escrito \(output.path): \(count) fotogramas a \(fps) fps")
