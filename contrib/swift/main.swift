
func run(){
  let kf = KalmanFilter()
  let index = [Double](repeating: 4, count: 10) 
  let data = index.map{ Double($0 * $0) }
  let k1 = data.map{ kf.filter($0) }

  let kf2 = KalmanFilter()
  let noisy = data.map{ $0 + Double.random(in: -2.1...5.8) }
  let k2 = data.map{ kf2.filter($0) }
  print("data : \(data)")
  print("noisy : \(noisy)")
  print("ideal : \(k1)")
  print("get : \(k2)")
}

run()
