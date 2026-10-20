<#
Creates the multi-resolution Windows icon embedded in AZP-Camera.exe (src/exe/resources.rc).
The PNG in assets/branding is the only icon source; the generated file stays in build/.
#>

param(
    [Parameter(Mandatory = $true)] [string] $Source,
    [Parameter(Mandatory = $true)] [string] $Destination
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Drawing

# One PNG-compressed frame of $Size x $Size pixels.
function ConvertTo-IconFrame {
    param([System.Drawing.Image] $Image, [int] $Size)

    $bitmap = [System.Drawing.Bitmap]::new($Size, $Size, [System.Drawing.Imaging.PixelFormat]::Format32bppArgb)
    $graphics = [System.Drawing.Graphics]::FromImage($bitmap)
    try {
        $graphics.Clear([System.Drawing.Color]::Transparent)
        $graphics.CompositingMode = [System.Drawing.Drawing2D.CompositingMode]::SourceCopy
        $graphics.CompositingQuality = [System.Drawing.Drawing2D.CompositingQuality]::HighQuality
        $graphics.InterpolationMode = [System.Drawing.Drawing2D.InterpolationMode]::HighQualityBicubic
        $graphics.SmoothingMode = [System.Drawing.Drawing2D.SmoothingMode]::HighQuality
        $graphics.PixelOffsetMode = [System.Drawing.Drawing2D.PixelOffsetMode]::HighQuality
        $graphics.DrawImage($Image, 0, 0, $Size, $Size)

        $stream = [System.IO.MemoryStream]::new()
        try {
            $bitmap.Save($stream, [System.Drawing.Imaging.ImageFormat]::Png)
            return ,$stream.ToArray()
        }
        finally {
            $stream.Dispose()
        }
    }
    finally {
        $graphics.Dispose()
        $bitmap.Dispose()
    }
}

# ICO header, one directory entry per frame, then the frames.
function Write-WindowsIcon {
    param([string] $Path, [int[]] $Sizes, [System.Collections.Generic.List[byte[]]] $Frames)

    $stream = [System.IO.File]::Create($Path)
    $writer = [System.IO.BinaryWriter]::new($stream)
    try {
        $writer.Write([uint16] 0)
        $writer.Write([uint16] 1)
        $writer.Write([uint16] $Frames.Count)

        $imageOffset = 6 + 16 * $Frames.Count
        for ($index = 0; $index -lt $Frames.Count; $index++) {
            $sizeByte = if ($Sizes[$index] -eq 256) { 0 } else { $Sizes[$index] }
            $writer.Write([byte] $sizeByte)
            $writer.Write([byte] $sizeByte)
            $writer.Write([byte] 0)
            $writer.Write([byte] 0)
            $writer.Write([uint16] 1)
            $writer.Write([uint16] 32)
            $writer.Write([uint32] $Frames[$index].Length)
            $writer.Write([uint32] $imageOffset)
            $imageOffset += $Frames[$index].Length
        }

        foreach ($frame in $Frames) {
            $writer.Write($frame)
        }
    }
    finally {
        $writer.Dispose()
        $stream.Dispose()
    }
}

$sourcePath = [System.IO.Path]::GetFullPath($Source)
$destinationPath = [System.IO.Path]::GetFullPath($Destination)
[System.IO.Directory]::CreateDirectory([System.IO.Path]::GetDirectoryName($destinationPath)) | Out-Null

$sizes = @(16, 24, 32, 48, 64, 128, 256)
$frames = [System.Collections.Generic.List[byte[]]]::new()
$sourceImage = [System.Drawing.Image]::FromFile($sourcePath)
try {
    foreach ($size in $sizes) {
        $frames.Add((ConvertTo-IconFrame -Image $sourceImage -Size $size))
    }
}
finally {
    $sourceImage.Dispose()
}

$temporaryPath = "$destinationPath.tmp"
Write-WindowsIcon -Path $temporaryPath -Sizes $sizes -Frames $frames
Move-Item -LiteralPath $temporaryPath -Destination $destinationPath -Force
