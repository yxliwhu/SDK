# Discription 
This version only includes the packaged framework which the user cannot view the source code.

# Usage
1. download and add to the project, refer to [Link](https://www.likecs.com/show-204439921.html)
2. coding example 
```
import SDK
...
...
  let myMianClass:mianClass
  myMianClass = .init()
  myMianClass.startCollectDataset()
  var timer = Timer.scheduledTimer(withTimeInterval: 1, repeats: true, block: { (timer) in
      let position = myMianClass.getLocationFormPolyU()
      print(position.coordinate)})
...
...
```
