class EyeController {
  constructor(elements = {}, eyeSize = '33.33vmin') {
    this._eyeSize = eyeSize;
    this._blinkTimeoutID = null;

    this.setElements(elements);
  }

  get leftEye() { return this._leftEye; }
  get rightEye() { return this._rightEye; }

  setElements({
    leftEye,
    rightEye,
    upperLeftEyelid,
    upperRightEyelid,
    lowerLeftEyelid,
    lowerRightEyelid,
    upperLeftMouthlid,
    upperRightMouthlid,
  } = {}) {
    this._leftEye = leftEye;
    this._rightEye = rightEye;
    this._upperLeftEyelid = upperLeftEyelid;
    this._upperRightEyelid = upperRightEyelid;
    this._lowerLeftEyelid = lowerLeftEyelid;
    this._lowerRightEyelid = lowerRightEyelid;
    this._upperLeftMouthlid = upperLeftMouthlid;
    this._upperRightMouthlid = upperRightMouthlid;
    return this;
  }

  _createKeyframes ({
    tgtTranYVal = 0,
    tgtRotVal = 0,
    enteredOffset = 1/3,
    exitingOffset = 2/3,
  } = {}) {
    return [
      {transform: `translateY(0px) rotate(0deg)`, offset: 0.0},
      {transform: `translateY(${tgtTranYVal}) rotate(${tgtRotVal})`, offset: enteredOffset},
      {transform: `translateY(${tgtTranYVal}) rotate(${tgtRotVal})`, offset: exitingOffset},
      {transform: `translateY(0px) rotate(0deg)`, offset: 1.0},
    ];
  }

  express({
    type = '',
    // level = 3,  // 1: min, 5: max
    duration = 1000,
    enterDuration = 75,
    exitDuration = 75,
  }) {
    if (!this._leftEye) {  // assumes all elements are always set together
      console.warn('Eye elements are not set; return;');
      return;
    }

    const options = {
      duration: duration,
    };
    
  }

  blink({
    duration = 150,  // in ms
  } = {}) {
    if (!this._leftEye) {  // assumes all elements are always set together
      console.warn('Eye elements are not set; return;');
      return;
    }

    [this._leftEye, this._rightEye].map((eye) => {
      eye.animate([
        {transform: 'rotateX(0deg)'},
        {transform: 'rotateX(90deg)'},
        {transform: 'rotateX(0deg)'},
      ], {
        duration,
        iterations: 1,
      });
    });
  }

  startBlinking({
    maxInterval = 5000
  } = {}) {
    if (this._blinkTimeoutID) {
      console.warn(`Already blinking with timeoutID=${this._blinkTimeoutID}; return;`);
      return;
    }
    const blinkRandomly = (timeout) => {
      this._blinkTimeoutID = setTimeout(() => {
        this.blink();
        blinkRandomly(Math.random() * maxInterval);
      }, timeout);
    }
    blinkRandomly(Math.random() * maxInterval);
  }
  
  normal() {
	    document.getElementById("soprdx").src = "image/eyebrowsx1.gif"; 
    	document.getElementById("soprsx").src = "image/eyebrowdx1.gif"; 
	    document.getElementById("occhiodx").src = "image/eyedx.png"; 
    	document.getElementById("occhiosx").src = "image/eyesx.png"; 	
    	document.getElementById("bocca").src = "image/boccastabile.png"; 
    	document.getElementById("naso").src = "image/nosetalkmarrtina1.gif"; 
  }	 
  
  happy() {
      document.getElementById("soprdx").src = "image/eyebrowsx1.gif"; 
      document.getElementById("soprsx").src = "image/eyebrowdx1.gif"; 
	    document.getElementById("occhiodx").src = "image/eyedx.png"; 
    	document.getElementById("occhiosx").src = "image/eyesx.png"; 
      document.getElementById("bocca").src = "image/boccaaperta.png"; 
     	document.getElementById("naso").src = "image/nose_happy.png"; 
  }	 

  sad() {
	document.getElementById("soprdx").src = "image/eyebrowdxangry.png"; 
    document.getElementById("soprsx").src = "image/eyebrowsxangry.png"; 
    document.getElementById("occhiodx").src = "image/eyedx.png"; 
    document.getElementById("occhiosx").src = "image/eyesx.png"; 	
    document.getElementById("bocca").src = "image/mouthangry.png"; 
    document.getElementById("naso").src = "image/nose_angry.png"; 
  }	 
  angry() {
	document.getElementById("soprdx").src = "image/eyebrowdxsad.png"; 
    document.getElementById("soprsx").src = "image/eyebrowsxsad.png"; 
    document.getElementById("occhiodx").src = "image/eyedx.png"; 
    document.getElementById("occhiosx").src = "image/eyesx.png"; 
    document.getElementById("bocca").src = "image/boccasad.png"; 
    document.getElementById("naso").src = "image/nose_sad.png"; 
  }	   
  surprised() {
	document.getElementById("soprdx").src = "image/eyebrowdxsurprise.png"; 
    document.getElementById("soprsx").src = "image/eyebrowsxsurprise.png"; 
    document.getElementById("occhiodx").src = "image/eyedx.png"; 
    document.getElementById("occhiosx").src = "image/eyesx.png"; 
    document.getElementById("bocca").src = "image/mouthsurprise.png"; 
    document.getElementById("naso").src = "image/nosesurprise.png"; 
  }
  speak() {
    document.getElementById("bocca").src = "image/talk3.gif"; 
    document.getElementById("naso").src = "image/nosetalkmarrtina1.gif"; 
  }
  sings() {
    document.getElementById("bocca").src = "image/talk3.gif"; 
    document.getElementById("naso").src = "image/nosetalkmarrtina1.gif"; 
  }
  stopBlinking() {
    clearTimeout(this._blinkTimeoutID);
    this._blinkTimeoutID = null;
  }

  setEyePosition(eyeElem, x, y, isRight = false) {
    if (!eyeElem) {  // assumes all elements are always set together
      console.warn('Invalid inputs ', eyeElem, x, y, '; retuning');
      return;
    }

    if (!!x) {
      if (!isRight) {
        eyeElem.style.left = `calc(${this._eyeSize} / 3 * 2 * ${x})`;
      } else {
        eyeElem.style.right = `calc(${this._eyeSize} / 3 * 2 * ${1-x})`;
      }
    }
    if (!!y) {
      eyeElem.style.bottom = `calc(${this._eyeSize} / 3 * 2 * ${1-y})`;
    }
  }
}

const eyes = new EyeController({
  leftEye: document.querySelector('.left.bulb'),
  rightEye: document.querySelector('.right.bulb'),
  upperLeftEyelid: document.querySelector('.left .eyelid.upper'),
  upperRightEyelid: document.querySelector('.right .eyelid.upper'),
  lowerLeftEyelid: document.querySelector('.left .eyelid.lower'),
  lowerRightEyelid: document.querySelector('.right .eyelid.lower'),
  upperLeftMouthlid: document.querySelector('.mouthid.upper'),
  lowerLeftMouthlid: document.querySelector('.mouthid.lower'),
  
});

