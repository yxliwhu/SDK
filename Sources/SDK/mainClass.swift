//
//  ViewController.swift
//  navigation
//
//  Created by 郑旭 on 2021/1/30.
//

import CoreLocation
import Foundation

public class mianClass{
    
    var inScanning:Bool!
    var isInitFunc:Bool!
    var beaconHelper = IBeaconHelper.shared
    var motionHelper = MotionHelper.shared
    
//    var curBeacon:iBeacon?
    var startTime:Int64 = 0
    var currentTime:Int64 = 0
    var indexPre:Int = 0
    var indexCurrent:Int = 0
    var gpsLocation:CLLocation?
    var trueHeaningMagn:CLHeading?
    var timer: Timer!
    var beaconFile:URL!
    var stepDetecor:StepDetector?
    var userHeight:String!
    var realTime:Bool = true
    var preKey:Int64 = 0
    
    var recordData: Bool = true
    
    var locationArrayGPS: [CLLocationCoordinate2D] = []
    var locationArrayFilter: [CLLocationCoordinate2D] = []
    var scanService: BeaconScanService? = nil
    
    public func start(){
        self.userHeight = "1.7"
        self.stepDetecor = StepDetector(self.userHeight,self.realTime)
        startCollectDataset()
    }
    public func getLocationFormPolyU() -> CLLocation{
        var kalmanFiterPos:CLLocation? = nil
        if let stp = self.stepDetecor {
            kalmanFiterPos = toMapLocation(stp.getDetector().getKalmanFiterPos())
        }
        return kalmanFiterPos!
    }
    
    
    func startCollectDataset(){
        self.motionProcess()
        self.bleProcess()
        self.gpsProcess()
    }

    /*
     The functions to process motion sensors
     */
    func motionProcess(){
        let storeFile = FileUtils.urlFile("SensorData")!
        // Here, get the sensor data from montionhelper then store to the file
        motionHelper.setMdBlock{ (pData) in
            if (self.recordData){
                let wsContent = self.getSensorWs(pData)
                FileUtils.writeStrings(storeFile, wsContent)
            }
            self.stepDetecor!.onSensorChanged(pData)// Step Counter 算法触发入口
        }
    }
    
    /*
     Get the data of motion sensors and store to "pData"
     */
    func getSensorWs(_ data:DeviceData)->[String]{
        var wsContent = [String]()
        let tNow = iBeaconClass.getNowMillis()
        let ts = "CurrentTime," + String(tNow)
        wsContent.append(ts)
        
        let accelerometer = "Accelerometer," + String(data.accelerometer!.acceleration.x) + "," + String(data.accelerometer!.acceleration.y) + "," + String(data.accelerometer!.acceleration.z)
        wsContent.append(accelerometer)
        
        let gyro = "GyroScope," + String(data.gyro!.rotationRate.x) + "," + String(data.gyro!.rotationRate.y) + "," + String(data.gyro!.rotationRate.z)
        
        wsContent.append(gyro)
        
        let magnetic = "Magnetic," + String(data.magnetic!.magneticField.x) + "," +
            String(data.magnetic!.magneticField.y) + "," + String(data.magnetic!.magneticField.z)
        wsContent.append(magnetic)
        
        let gravity = "Motion-Gravity," + String(data.motion!.gravity.x) + "," + String(data.motion!.gravity.y) + "," + String(data.motion!.gravity.z)
        
        wsContent.append(gravity)
        
        let attitude = "Motion-Attitude," + String(data.motion!.attitude.pitch) + "," + String(data.motion!.attitude.roll) + "," + String(data.motion!.attitude.yaw)
        
        wsContent.append(attitude)
        
        let rotationRate = "Motion-RotationRate," + String(data.motion!.rotationRate.x) + "," +
            String(data.motion!.rotationRate.y) + "," + String(data.motion!.rotationRate.z)
        
        if let tlocation = self.gpsLocation {
            let gpsInfo = "GPS," + String(tlocation.coordinate.latitude) + "," + String(tlocation.coordinate.longitude) + "," + String(tlocation.altitude)
            wsContent.append(gpsInfo)
        }
        
        wsContent.append(rotationRate)
        return wsContent
    }
    
    /*
     The functions to process GPS sensor
     */
    func gpsProcess(){
        let storeFile = FileUtils.urlFile("GpsData")!
        beaconHelper.setGpsDataBlock{ (pData) in
            if (self.recordData){
                let wsContent = self.getGpsWs(pData)
                FileUtils.writeStrings(storeFile, wsContent)
            }
            self.gpsLocation = pData
        }
        beaconHelper.setHeadingDataBlock{ (pData) in
            self.trueHeaningMagn = pData
            self.stepDetecor?.kalmanPositionDetector.setMagnHeading(self.trueHeaningMagn!.trueHeading)
        }
    }
    /*
     Get the data of GPS and sotre to "pData"
     */
    func getGpsWs(_ data:CLLocation)->[String]{
        var wsContent = [String]()
        let tNow = Int64(data.timestamp.timeIntervalSince1970 * 1000)

        let ts = "CurrentTime," + String(tNow)
        wsContent.append(ts)
        
        let position = "Latitude:" + String(data.coordinate.latitude) + ", Longitude" + String(data.coordinate.longitude)
        wsContent.append(position)
        self.stepDetecor?.getDetector().data.setGPSlocation(data, tNow)
        return wsContent
    }
    
    /*
     The functions to process BLE sensor
     */
    func bleProcess(){
        self.beaconFile = FileUtils.urlFile("BeaconData")!
        //Record the BLE data every 0.5 second
        if (self.realTime) {
            self.timer = Timer.scheduledTimer(timeInterval: 0.5, target: self, selector: #selector(insertBeaconData), userInfo: nil, repeats: true)
        }
        beaconHelper.setBackBeaconBlock { (pData) in
            if (self.startTime == 0) {
                self.startTime = iBeaconClass.getNowMillis()
            }
            self.currentTime = iBeaconClass.getNowMillis()
            let seqTime = self.currentTime - self.startTime
            
            self.indexCurrent = Int(floor(Double(seqTime) / 1000.0))
            let indexDelta = self.indexCurrent - self.indexPre
            
            if (indexDelta >= 1 && indexDelta < 2) {
                self.indexPre = self.indexCurrent
            }
            if(indexDelta >= 2){
                let tNow = iBeaconClass.getNowMillis()
                let tq = tNow - self.startTime
                let tempIndex = Int(floor(Double(tq) / 1000.0))
                self.indexPre = tempIndex
            }
            let beaconScanStartTime:Int64 = Int64(Int(self.startTime) + 1000 * self.indexCurrent + 500)
            for tIBeacon in pData {
                var m_beacon = self.toCustomIbeacon(tIBeacon)
                // Sometimes, the record of rssi is zero, here we treat it as -99 (very week signal)
                if (m_beacon.rssi == 0){
                    m_beacon.rssi = -99
                }
                if (self.indexCurrent < 1){
                    self.scanService = BeaconScanService(m_beacon,self.startTime,self.indexCurrent)
                }else{
                    
                }
                if (self.recordData){
                    self.recordScanningData(m_beacon)
                }
                self.stepDetecor?.kalmanPositionDetector.BeaconUsedRecord = true
                self.stepDetecor?.kalmanPositionDetector.setBeaconUsed(m_beacon)
                self.stepDetecor?.kalmanPositionDetector.initSensorChange()
                self.stepDetecor?.kalmanPositionDetector.calculate(indexUsed: 0)
                
                
                
                // ******modified 2022.05.09: find this code is no used
                self.scanService!.StoreSignalPeakDetection(m_beacon, self.currentTime)
                self.scanService!.SignalPeak(self.currentTime)
                // ******modified 2022.05.09: find this code is no use
                self.scanService!.UsedputtempBeaconAverageValues(m_beacon.minor, beaconScanStartTime, Double(m_beacon.rssi))
                

                let KeyDistance:[Int64:[Double]] = BeaconPositioningAlgorithm.CalculateDistanceMapByStrongBeacon(m_beacon, self.scanService!.StoreScannedBeaconStrong)
                if (KeyDistance.count >= 3){
                    self.scanService!.StoreScannedBeaconStrong.removeAll()
                    let StrongBeaconPosXYTemp = BeaconPositioningAlgorithm.CalculatePositionByDistance(KeyDistance)
                    let StrongBeaconPosXY:[Double] =  [StrongBeaconPosXYTemp[0], StrongBeaconPosXYTemp[1], StrongBeaconPosXYTemp[2], StrongBeaconPosXYTemp[3]]
                    self.stepDetecor!.getDetector().setStrongBeaconPosXY(StrongBeaconPosXY)
                }
            }
            self.ibeaconScanDataProcess(Int64(indexDelta), beaconScanStartTime)
            
        }
    }

    
    /*
     Format the stored BLE file (add nil value to the time gap)
     */
    @objc func insertBeaconData(){
        guard self.inScanning else {
            return
        }
        let nTime = iBeaconClass.getNowMillis()
        let seqTime = nTime - self.currentTime
        if (seqTime >= 500){
            var wsContent = [String]()
            let line = "postEveryBeacon," + "0" + "," + "0" + "," + String(self.startTime) + "," + String(nTime) + "," + String(self.indexPre) + "," + "0"
            wsContent.append(line)
            
//            FileUtils.writeStrings(self.beaconFile, wsContent)
        }
    }
    
    /*
     Transform the format of BLE samples
     Here merge the major and minor to the minor value: 0-4 is major 5-9 is minor
     */
    func toCustomIbeacon(_ ibc: CLBeacon)->iBeacon{
        var ibeacon = iBeacon()
        ibeacon.major = ibc.major.intValue
        ibeacon.minor = Int64(ibc.major.intValue * 100000) + ibc.minor.int64Value
        ibeacon.proximityUuid = ibc.uuid.uuidString
        ibeacon.rssi = ibc.rssi
        ibeacon.distance = String(format: "%.2f", ibc.accuracy)
        return ibeacon
    }
    
    /*
     Function to scan and process the BLE
     */
    func ibeaconScanDataProcess(_ indexDelta:Int64, _ beaconScanStartTime:Int64){
        let beaconScanStartTime:Int64 = Int64(Int(self.startTime) + 1000 * self.indexCurrent + 500)
        
        if (indexDelta >= 1 && indexDelta < 2) {
            self.scanService!.updateAverageValues(beaconScanStartTime)
            self.scanService!.BuildStrongBeaconMap()
            self.scanService!.GetNonZeroMap(self.scanService!.StrongBeacon)
            self.scanService!.CalculateSlope(self.scanService!.NonZeroStrongMap, self.scanService!.StrongBeacon)
            self.scanService!.StrongBeaconKeyIndicator = CalculateIndicator.StrongIndicator(self.scanService!.minorSlope, &self.scanService!.StrongBeaconKeyIndicator, self.scanService!.StrongBeaconSlopeIndexPlus, self.scanService!.StrongBeaconSlopeIndexMius)
            
            self.scanService!.CalculateWeekBeaconIndicator()
            self.scanService!.MergeHeadingANDClearData()
            self.scanService!.SendHeadingToActivity()
            
            if(!self.scanService!.HeadingIndex.isEmpty && self.scanService!.HeadingIndex[0] != 800){
                stepDetecor!.getDetector().setBeaconHeadVals(Float(self.scanService!.HeadingIndex[0]), self.scanService!.HeadingIndex[1], self.scanService!.HeadingIndex[2])
                self.scanService!.HeadingIndex[0] = 800
            }
        }
    }
    
    /*
     Store the BLE data to the storage
     */
    func recordScanningData(_ m_beacon: iBeacon){
        var wsContent = [String]()
        let line = self.getWsLine(m_beacon.major, m_beacon.minor, m_beacon.rssi, self.startTime, self.currentTime, self.indexPre)
        wsContent.append(line)
        FileUtils.writeStrings(self.beaconFile, wsContent)
    }
    
    /*
     Create string line to be storage for BLE samples
     */
    func getWsLine(_ major:Int,
                   _ minor:Int64,
                   _ rssi:Int,
                   _ start:Int64,
                   _ current:Int64,
                   _ index:Int)->String{
        
        let c_minor = String(major) + String(minor)
        return "postEveryBeacon," + c_minor + "," + String(rssi) + "," + String(start) + "," + String(current) + "," + String(index) + "," + "0"
    }
    
    public func printTime(){
        let date = Date()
        let timeFormatter = DateFormatter()
        timeFormatter.dateFormat = "yyy-MM-dd' at 'HH:mm:ss.SSS"
        let strNowTime = timeFormatter.string(from: date) as String
        print(strNowTime)
    }
    
    /*
     Transform the location to "CLLocation" format
     */
    func toMapLocation(_ location: LatLng) -> CLLocation{
        return CLLocation(latitude: location.latitude, longitude: location.longitude)
    }
    
}
