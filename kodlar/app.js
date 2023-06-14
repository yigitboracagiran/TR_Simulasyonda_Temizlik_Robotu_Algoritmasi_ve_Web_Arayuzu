const ros = new ROSLIB.Ros(); //ROS'a bağlanma

ros.on('error', function(error) {
  console.error('Error connecting to ROS: ', error);
});

ros.connect('ws://localhost:9090');

var compressedImageTopic = new ROSLIB.Topic({ //Aracın kamera verisine abone olma
    ros: ros,
    name : '/camera/rgb/image_raw/compressed',
    messageType : 'sensor_msgs/CompressedImage'
});

compressedImageTopic.subscribe(function(message) { //Kamera verisini arayüzde yayınlama
    var imageData = "data:image/jpeg;base64," + message.data;
    var image = new Image();
    image.src = imageData;
    image.onload = function() {
        var canvas = document.getElementById("canvas");
        canvas.width = 630;
        canvas.height = 460;
        if(canvas) {
            var context = canvas.getContext("2d");
            context.drawImage(image, 0, 0);
        }
    };
});

var topic = new ROSLIB.Topic({ //Aracın hangi modda olduğunu tutan topic
    ros : ros,
    name : '/modKontrolu',
    messageType : 'std_msgs/String'
});

var message = new ROSLIB.Message({ //Herhangi bir modda değil -> -1
    data : '-1'
});
topic.publish(message);

var kontrol = -1; //Aracın hangi modda olduğunu tutan değişken

var manuelSurusButton = document.getElementById("manuelSurusButton"); //Modu değiştiren butona her tıklandığında butonun rengini değiştirmek için classList'inde oynama yapılıyor.
manuelSurusButton.addEventListener("click", function() {

    if( kontrol != -2 ){
        if (manuelSurusButton.classList.contains("clicked1")) {
            manuelSurusButton.classList.remove("clicked1");
            manuelSurusButton.classList.add("clicked2");
            manuelSurusButton.classList.remove("clicked3");
        }
        else if (manuelSurusButton.classList.contains("clicked2")) {
            manuelSurusButton.classList.remove("clicked1");
            manuelSurusButton.classList.remove("clicked2");
            manuelSurusButton.classList.add("clicked3");
        }
        else if (manuelSurusButton.classList.contains("clicked3")) {
            manuelSurusButton.classList.remove("clicked1");
            manuelSurusButton.classList.remove("clicked2");
            manuelSurusButton.classList.remove("clicked3");
        }
        else {
            manuelSurusButton.classList.add("clicked1");
        }
    }

    if( kontrol == 0 ) { //classList'i değiştikten sonra da modu tutan topic ve değişken değiştiriliyor.
        var message = new ROSLIB.Message({
            data : '1'
        });
        topic.publish( message );
        kontrol = 1;
    }
    else if( kontrol == 1 ) {
        var message = new ROSLIB.Message({
            data : '2'
        });
        topic.publish( message );
        kontrol = 2;
    }
    else if( kontrol == 2 ) {
        var message = new ROSLIB.Message({
            data : '-1'
        });
        topic.publish( message );
        kontrol = -1;
    }
    else if( kontrol == -1 ) {
        var message = new ROSLIB.Message({
            data : '0'
        });
        topic.publish( message );
        kontrol = 0;
    }

});

var twistTopic = new ROSLIB.Topic({ //Abone olunan hız topic'i
    ros: ros,
    name: '/cmd_vel1',
    messageType: 'geometry_msgs/Twist'
});

var newTwistTopic = new ROSLIB.Topic({ //Python koduna gönderilen hız topic'i
    ros: ros,
    name : '/cmd_vel',
    messageType : 'geometry_msgs/Twist'
});

val_linear_x = 0
val_angular_z = 0

newTwistTopic.subscribe(function(message) { //Abone olunan topic'ten gelen hız verileri değişkenlere aktarılıyor.
    val_linear_x = message.linear.x
    val_angular_z = message.angular.z
});

var acilDurdurma = document.getElementById("acilDurdurma"); //Acil Durdurma butonuna tıklandığında araç durur ve mod butonu başlangıca döner, 2. defa acil durdurma butonuna basmadan mod değiştirilemez.
acilDurdurma.addEventListener("click", function() {
    kontrol = -2;
    manuelSurusButton.classList.remove("clicked1");
    manuelSurusButton.classList.remove("clicked2");
    manuelSurusButton.classList.remove("clicked3");
    if ( acilDurdurma.classList.contains("clicked") ) {
        acilDurdurma.classList.remove("clicked");
        kontrol = -1;
    }
    else {
        acilDurdurma.classList.add("clicked");
    }
    if ( kontrol == -1 ){
        var message = new ROSLIB.Message({
            data : '-1'
        });
    }
    if ( kontrol == -2 ){
        var message = new ROSLIB.Message({
            data : '-2'
        });
    }
    topic.publish( message );
    var twist = new ROSLIB.Message({
        linear : {
            x : 0
        },
        angular : {
            z : 0
        }
    });
    twistTopic.publish( twist )
});

var ileriButton = document.getElementById("ileriButton"); //Arayüzdeki haraket butonları
var sagButton = document.getElementById("sagButton");
var solButton = document.getElementById("solButton");
var geriButton = document.getElementById("geriButton");
var durButton = document.getElementById("durButton");

ileriButton.addEventListener("click", function() { //İleri butona tıklandığında lineer hız artar.
    if( kontrol == 1 ){
        val_linear_x += 0.02
        var twist = new ROSLIB.Message({
            linear : {
                x : val_linear_x
            },
            angular : {
                z : val_angular_z
            }
        });
        twistTopic.publish( twist )
    }
});

sagButton.addEventListener("click", function() { //Geri butona tıklandığında lineer hız azalır.
    if( kontrol == 1 ){
        val_angular_z -= 0.02
        var twist = new ROSLIB.Message({
            linear : {
                x : val_linear_x
            },
            angular : {
                z : val_angular_z
            }
        });
        twistTopic.publish( twist )
    }
});

solButton.addEventListener("click", function() { //Sol butona tıklandığında açısal hız artar.
    if( kontrol == 1 ){
        val_angular_z +=0.02
        var twist = new ROSLIB.Message({
            linear : {
                x : val_linear_x
            },
            angular : {
                z : val_angular_z
            }
        });
        twistTopic.publish( twist )
    }
});

geriButton.addEventListener("click", function() { //Sağ butona tıklandığında açısal hız azalır.
    if( kontrol == 1 ){
        val_linear_x -= 0.02
        var twist = new ROSLIB.Message({
            linear : {
                x : val_linear_x
            },
            angular : {
                z : val_angular_z
            }
        });
        twistTopic.publish( twist )
    }
});

durButton.addEventListener("click", function() { //Dur butona basıldığında açısal ve lineer hız sıfırlanır.
    if( kontrol == 1 ){
        var twist = new ROSLIB.Message({
            linear : {
                x : 0
            },
            angular : {
                z : 0
            }
        });
        twistTopic.publish( twist )
   }
});


var noktaTopic = new ROSLIB.Topic({ //Arayüz modundayken haritaya tıklandıldığında tıklanılan piksel bu topic ile yayınlanır.
    ros : ros,
    name: '/clicked_point',
    messageType: 'geometry_msgs/Point'
});

const image = document.getElementById('image');
const canvas2 = document.getElementById('canvas2')

image.addEventListener('click', function( event ) { //Tıklanılan piksel topic ile yayınlanıyor.
    const rect = image.getBoundingClientRect();
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;
    const point = new ROSLIB.Message({
        x: x,
        y: y,
        z: 0
    });
    noktaTopic.publish( point );
});

const okTopic = new ROSLIB.Topic({ //Aracın anlık konumu bu topic ile geliyor.
    ros: ros,
    name: '/ok',
    messageType: 'geometry_msgs/Point'
});

okTopic.subscribe( function ( message ) { //Aracın konumu koordinattan piksele dönüştürülüyor ve haritada yayınlanıyor. Ardından geçtiği piksel kırmızıya boyanıyor.
    const arrow = document.getElementById('arrow');
    arrow.style.display = 'block';
    const rect = image.getBoundingClientRect();
    arrow.style.left = Math.round( message.x ) + rect.left + 'px';
    arrow.style.top = Math.round( message.y ) + rect.top + 'px';
    const degree = message.z
    arrow.style.transform = `rotate(${degree}deg)`;
    canvas2.width = image.width;
    canvas2.height = image.height;
    var ctx = canvas2.getContext('2d');
    ctx.drawImage( image, 0, 0 );
    const imageData = ctx.getImageData( 0, 0, canvas2.width, canvas2.height );
    const pixelData = imageData.data;
    const pixelIndex = ( ( Math.round( message.y ) * canvas2.width ) + Math.round( message.x ) ) * 4;
    pixelData[pixelIndex] = 255; // Kirmizi
    pixelData[pixelIndex + 1] = 0; // Yesil
    pixelData[pixelIndex + 2] = 0; // Mavi
    ctx.putImageData( imageData, 0, 0 );
    image.src = canvas2.toDataURL();
});